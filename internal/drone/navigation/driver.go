package navigation

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/sensors"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

// Driver соединяет карту с ограниченными позиционными целями; все методы вызывает один исполнитель.
type Driver struct {
	config       Config
	grid         *Grid
	sensors      sensors.Reader
	states       telemetry.StateReader
	positions    route.PositionController
	velocity     route.VelocityController
	logger       *slog.Logger
	origin       telemetry.Position
	offset       Point
	session      string
	lastSent     time.Time
	blockedSince time.Time
	speedSet     bool
	lastLogged   time.Time
}

// NewDriver фиксирует общую привязку GPS/сенсоров до arm и проверяет размер карты.
func NewDriver(
	config Config,
	bounds Bounds,
	reader sensors.Reader,
	states telemetry.StateReader,
	positions route.PositionController,
	velocity route.VelocityController,
	origin *telemetry.Position,
	logger *slog.Logger,
) (*Driver, error) {
	if reader == nil || states == nil || positions == nil || velocity == nil || logger == nil ||
		!coordinates.ValidPosition(origin) ||
		math.Abs(origin.LatitudeDeg) > 85 {
		return nil, errors.New("navigation dependency missing")
	}
	if _, ok := positions.(route.NavigationSpeedController); !ok {
		return nil, errors.New("navigation requires confirmed autopilot speed limit")
	}
	g, err := NewGrid(bounds, config)
	if err != nil {
		return nil, err
	}
	f := reader.Snapshot()
	if !f.Fresh(time.Now(), config.SensorMaxAge) || f.Validate() != nil || f.SystemID != origin.SystemID {
		return nil, errors.New("fresh lidar and sonar frame required before flight")
	}
	if f.Lidar.MaxRangeM < config.LookaheadM+2*config.ClearanceM {
		return nil, errors.New("lidar range shorter than navigation envelope")
	}
	return &Driver{config: config, grid: g, sensors: reader, states: states, positions: positions, velocity: velocity,
		logger: logger, origin: *origin, offset: Point{-f.Pose.NorthM, -f.Pose.EastM}, session: f.Session}, nil
}

// SetPositionTarget сохраняет совместимость с обычным контроллером; маршрут использует FollowTarget.
func (d *Driver) SetPositionTarget(ctx context.Context, target route.PositionTarget) error {
	return d.positions.SetPositionTarget(ctx, target)
}

func (d *Driver) local(latitude, longitude float64) Point {
	return Point{(latitude - d.origin.LatitudeDeg) * coordinates.MetersPerDegree,
		math.Remainder(
			longitude-d.origin.LongitudeDeg,
			360,
		) * coordinates.MetersPerDegree * math.Cos(
			d.origin.LatitudeDeg*math.Pi/180,
		)}
}

func (d *Driver) stop(ctx context.Context) error { return d.velocity.SetVelocityTarget(ctx, 0, 0, 0) }

func (d *Driver) step(
	ctx context.Context,
	before telemetry.State,
	config route.RouteCheckConfig,
	target route.PositionTarget,
) error {
	state := d.states.Snapshot()
	if err := flight.CheckState(state, before, config.Flight, true); err != nil {
		_ = d.stop(ctx)
		return err
	}
	f := d.sensors.Snapshot()
	if !f.Fresh(time.Now(), d.config.SensorMaxAge) || f.Session != d.session || f.SystemID != before.SystemID ||
		f.Validate() != nil {
		_ = d.stop(ctx)
		return errors.New("navigation sensors stale, invalid or restarted")
	}
	if math.Abs(f.Pose.RollRad) > d.config.MaxTiltRad || math.Abs(f.Pose.PitchRad) > d.config.MaxTiltRad {
		_ = d.stop(ctx)
		return errors.New("navigation attitude exceeds horizontal scan limits")
	}
	current := Point{f.Pose.NorthM + d.offset.NorthM, f.Pose.EastM + d.offset.EastM}
	if distance(current, d.local(state.Position.LatitudeDeg, state.Position.LongitudeDeg)) > d.config.PoseToleranceM {
		_ = d.stop(ctx)
		return errors.New("navigation sensor/GPS position disagreement")
	}
	for i, s := range f.Sonars {
		if i != 9 && s.RangeM < d.config.EmergencyM {
			_ = d.stop(ctx)
			return errors.New("navigation emergency sonar clearance")
		}
	}
	if err := d.grid.Observe(f, d.offset); err != nil {
		_ = d.stop(ctx)
		return err
	}
	goal := d.local(target.LatitudeDeg, target.LongitudeDeg)
	carrot, err := d.grid.Carrot(current, goal)
	if err != nil {
		if stopErr := d.stop(ctx); stopErr != nil {
			return errors.Join(err, stopErr)
		}
		if d.blockedSince.IsZero() {
			d.blockedSince = time.Now()
			d.logger.Info("navigation holding for path", "target", target)
		}
		if time.Since(d.blockedSince) >= 3*time.Second {
			d.logger.Warn("navigation path unavailable", "current", current, "goal", goal, "frame", f,
				"mapCells", d.grid.cells, "mapOrigin", d.grid.origin, "width", d.grid.width, "height", d.grid.height)
			return fmt.Errorf("coverage incomplete: %w", err)
		}
		return nil
	}
	d.blockedSince = time.Time{}
	// Позиционный контур автопилота удерживает короткую цель; скорость ограничена подтверждённой командой.
	longitude := d.origin.LongitudeDeg + carrot.EastM/(coordinates.MetersPerDegree*math.Cos(d.origin.LatitudeDeg*math.Pi/180))
	longitude = math.Mod(longitude+540, 360) - 180
	command := route.PositionTarget{LatitudeDeg: d.origin.LatitudeDeg + carrot.NorthM/coordinates.MetersPerDegree,
		LongitudeDeg: longitude, RelativeAltitudeM: target.RelativeAltitudeM}
	if err := d.positions.SetPositionTarget(ctx, command); err != nil {
		return err
	}
	if d.lastLogged.IsZero() || time.Since(d.lastLogged) >= time.Second {
		d.logger.Info("navigation target", "current", current, "goal", goal, "carrot", carrot, "target", command)
		d.lastLogged = time.Now()
	}
	d.lastSent = time.Now()
	return nil
}

// FollowTarget перепланирует короткую команду на каждом новом такте, сохраняя исходную цель покрытия.
func (d *Driver) FollowTarget(
	ctx context.Context,
	before telemetry.State,
	config route.RouteCheckConfig,
	target route.PositionTarget,
) error {
	if err := target.Validate(); err != nil {
		return err
	}
	if !d.speedSet {
		if err := d.positions.(route.NavigationSpeedController).SetNavigationSpeed(ctx, d.config.MaxSpeedMS); err != nil {
			return err
		}
		d.speedSet = true
	}
	d.blockedSince = time.Time{}
	started := time.Now()
	var arrived time.Time
	ticker := time.NewTicker(d.config.UpdateInterval)
	defer ticker.Stop()
	for {
		if err := ctx.Err(); err != nil {
			_ = d.stop(context.WithoutCancel(ctx))
			return err
		}
		if err := d.step(ctx, before, config, target); err != nil {
			return err
		}
		p := d.states.Snapshot().Position
		if p != nil && p.ReceivedAt.After(started) && route.NearTarget(p, target, config) && d.blockedSince.IsZero() {
			if arrived.IsZero() {
				arrived = p.ReceivedAt
			}
			if p.ReceivedAt.Sub(arrived) >= config.ArrivalDuration {
				return nil
			}
		} else {
			arrived = time.Time{}
		}
		select {
		case <-ctx.Done():
		case <-ticker.C:
		}
	}
}

// HoldTarget поддерживает удержание во время сетевого барьера, продолжая проверять сенсоры.
func (d *Driver) HoldTarget(
	ctx context.Context,
	before telemetry.State,
	config route.RouteCheckConfig,
	target route.PositionTarget,
) error {
	if !d.lastSent.IsZero() && time.Since(d.lastSent) < d.config.UpdateInterval {
		return nil
	}
	return d.step(ctx, before, config, target)
}
