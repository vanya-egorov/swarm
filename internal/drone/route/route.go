package route

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	flightcore "github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type State = telemetry.State
type StateReader = telemetry.StateReader
type Position = telemetry.Position
type FlightMode = telemetry.FlightMode
type FlightCheckConfig = flightcore.FlightCheckConfig
type FlightController = flightcore.FlightController
type ArmingController = flightcore.ArmingController

const (
	ModeStabilize = telemetry.ModeStabilize
	ModeGuided    = telemetry.ModeGuided
	ModeLand      = telemetry.ModeLand
)

// Waypoint is a north/east offset from the launch point and an altitude above home.
type Waypoint struct{ NorthM, EastM, AltitudeM float64 }

// PositionTarget contains WGS84 coordinates and altitude above home.
type PositionTarget struct{ LatitudeDeg, LongitudeDeg, RelativeAltitudeM float64 }

// PositionController sends a target; telemetry confirms arrival.
type PositionController interface {
	SetPositionTarget(context.Context, PositionTarget) error
}

// RunRouteCheck проверяет весь маршрут до arm и сохраняет посадку при любой ошибке.
func RunRouteCheck(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	positions PositionController,
	states StateReader,
	logger *slog.Logger,
	config RouteCheckConfig,
) error {
	if positions == nil || flight == nil || arming == nil || states == nil || logger == nil {
		return errors.New("route check: missing dependency")
	}
	if err := config.Validate(); err != nil {
		return fmt.Errorf("route config: %w", err)
	}
	readyCtx, cancel := context.WithTimeout(ctx, config.Flight.ReadyTimeout)
	origin, err := flightcore.WaitForState(
		readyCtx,
		states,
		func(s State) bool { return flightcore.FreshState(s, config.Flight.TelemetryMaxAge) },
	)
	cancel()
	if err != nil {
		return fmt.Errorf("route origin: %w", err)
	}
	targets, err := TargetsFrom(origin.Position, config.Waypoints)
	if err != nil {
		return err
	}
	return flightcore.RunFlightCheckTask(ctx, flight, arming, states, logger, config.Flight, func(before State) error {
		// Место старта не должно измениться между подготовкой и взлётом.
		if !flightcore.SameDrone(origin, before) ||
			coordinates.HorizontalDistance(
				origin.Position.LatitudeDeg,
				origin.Position.LongitudeDeg,
				before.Position.LatitudeDeg,
				before.Position.LongitudeDeg,
			) > config.HorizontalToleranceM {
			return errors.New("route origin changed")
		}
		// Горизонтальное движение начинается после стабилизации подъёма.
		settleCtx, cancel := context.WithTimeout(ctx, config.Flight.ClimbTimeout)
		err := followTarget(settleCtx, positions, states, before, config, PositionTarget{
			LatitudeDeg:       before.Position.LatitudeDeg,
			LongitudeDeg:      before.Position.LongitudeDeg,
			RelativeAltitudeM: config.Flight.TargetAltitudeM,
		})
		cancel()
		if err != nil {
			return fmt.Errorf("hover: %w", err)
		}
		logger.Info("hover stable", "systemId", before.SystemID)
		for i, target := range targets {
			logger.Info("waypoint started", "index", i+1, "target", target)
			waypointCtx, cancel := context.WithTimeout(ctx, config.WaypointTimeout)
			err := followTarget(waypointCtx, positions, states, before, config, target)
			cancel()
			if err != nil {
				return fmt.Errorf("waypoint %d: %w", i+1, err)
			}
			logger.Info("waypoint reached", "index", i+1)
		}
		return nil
	})
}

func followTarget(
	ctx context.Context,
	positions PositionController,
	states StateReader,
	before State,
	config RouteCheckConfig,
	target PositionTarget,
) error {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	var firstSent, lastSent, arrived time.Time
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		state := states.Snapshot()
		if err := flightcore.CheckState(state, before, config.Flight, true); err != nil {
			return err
		}
		if lastSent.IsZero() || time.Since(lastSent) >= time.Second {
			sent := time.Now()
			if err := flightcore.RunCommand(ctx, config.Flight.CommandTimeout, func(c context.Context) error { return positions.SetPositionTarget(c, target) }); err != nil {
				return err
			}
			lastSent = sent
			if firstSent.IsZero() {
				firstSent = sent
			}
		}
		p := state.Position
		atTarget := p.ReceivedAt.After(firstSent) && NearTarget(p, target, config)
		if atTarget {
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
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

func NearTarget(p *Position, target PositionTarget, config RouteCheckConfig) bool {
	return coordinates.HorizontalDistance(
		p.LatitudeDeg,
		p.LongitudeDeg,
		target.LatitudeDeg,
		target.LongitudeDeg,
	) <= config.HorizontalToleranceM &&
		math.Abs(p.RelativeAltitudeM-target.RelativeAltitudeM) <= config.AltitudeToleranceM &&
		math.Hypot(p.NorthVelocityMS, p.EastVelocityMS) < 0.3 &&
		math.Abs(p.DownVelocityMS) < 0.3
}

// Validate проверяет координаты и допустимую высоту цели SITL.
func (p PositionTarget) Validate() error {
	if !coordinates.Finite(p.LatitudeDeg) || math.Abs(p.LatitudeDeg) > 90 || !coordinates.Finite(p.LongitudeDeg) ||
		math.Abs(p.LongitudeDeg) > 180 ||
		!coordinates.Finite(p.RelativeAltitudeM) ||
		p.RelativeAltitudeM < 0.8 ||
		p.RelativeAltitudeM > 2 {
		return errors.New("invalid position target")
	}
	return nil
}

func TargetsFrom(origin *telemetry.Position, waypoints []Waypoint) ([]PositionTarget, error) {
	if !coordinates.ValidPosition(origin) || math.Abs(origin.LatitudeDeg) > 85 {
		return nil, errors.New("invalid route origin (latitude must be within ±85 degrees)")
	}
	targets := make([]PositionTarget, len(waypoints))
	for i, p := range waypoints {
		lon := origin.LongitudeDeg + p.EastM/(coordinates.MetersPerDegree*math.Cos(origin.LatitudeDeg*math.Pi/180))
		lon = math.Mod(lon+540, 360) - 180
		targets[i] = PositionTarget{origin.LatitudeDeg + p.NorthM/coordinates.MetersPerDegree, lon, p.AltitudeM}
		if err := targets[i].Validate(); err != nil {
			return nil, err
		}
	}
	return targets, nil
}
