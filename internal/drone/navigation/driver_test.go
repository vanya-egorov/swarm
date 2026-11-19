package navigation

import (
	"context"
	"errors"
	"log/slog"
	"math"
	"testing"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/sensors"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type driverFake struct {
	point                       Point
	sequence                    uint64
	north, east                 float64
	stale, restarted, disagreed bool
	commands                    int
	minimum                     float64
	box                         Bounds
}

func (f *driverFake) Snapshot() telemetry.State {
	now := time.Now()
	return telemetry.State{SystemID: 1, ComponentID: 1, Armed: true, Mode: telemetry.ModeGuided, LastHeartbeat: now,
		Position: &telemetry.Position{
			SystemID:          1,
			ComponentID:       1,
			LatitudeDeg:       (f.point.NorthM - 2) / coordinates.MetersPerDegree,
			LongitudeDeg:      (f.point.EastM - 5) / coordinates.MetersPerDegree,
			RelativeAltitudeM: 1.5,
			NorthVelocityMS:   f.north,
			EastVelocityMS:    f.east,
			ReceivedAt:        now,
		},
		ServoOutputs: &telemetry.ServoOutputs{SystemID: 1, ComponentID: 1, ReceivedAt: now}}
}

type driverSensors struct{ fake *driverFake }

func (r driverSensors) Snapshot() *sensors.Frame {
	f := r.fake
	f.sequence++
	frame := scanAt(f.point, []Bounds{f.box}, f.sequence)
	if f.stale {
		frame.ReceivedAt = time.Now().Add(-time.Hour)
	}
	if f.restarted {
		frame.Session = "other"
	}
	if f.disagreed {
		frame.Pose.NorthM += 3
	}
	return frame
}
func (f *driverFake) SetPositionTarget(ctx context.Context, target route.PositionTarget) error {
	goal := Point{
		target.LatitudeDeg*coordinates.MetersPerDegree + 2,
		target.LongitudeDeg*coordinates.MetersPerDegree + 5,
	}
	n, e := goal.NorthM-f.point.NorthM, goal.EastM-f.point.EastM
	length := math.Hypot(n, e)
	if length > 0.50001 {
		return errors.New("unbounded position command bypassed local planner")
	}
	speed := math.Min(0.5, length)
	if length > 1e-9 {
		n, e = n*speed/length, e*speed/length
	}
	return f.SetVelocityTarget(ctx, n, e, 0)
}

func (f *driverFake) SetNavigationSpeed(_ context.Context, speed float64) error {
	if speed > 0.5 {
		return errors.New("unsafe speed")
	}
	return nil
}
func (f *driverFake) SetVelocityTarget(_ context.Context, n, e, d float64) error {
	if math.Hypot(n, e) > 0.50001 || math.Abs(d) > 0.25001 {
		return errors.New("speed limit exceeded")
	}
	f.commands++
	f.north, f.east = n, e
	f.point.NorthM += n * 0.1
	f.point.EastM += e * 0.1
	f.minimum = math.Min(f.minimum, clearance(f.point, f.box))
	return nil
}

func newTestDriver(t *testing.T) (*Driver, *driverFake, route.RouteCheckConfig) {
	t.Helper()
	f := &driverFake{point: Point{2, 5}, box: Bounds{Min: Point{5, 3.5}, Max: Point{6, 6.5}}, minimum: 100}
	config := DefaultConfig()
	config.UpdateInterval = 20 * time.Millisecond
	origin := f.Snapshot().Position
	d, err := NewDriver(
		config,
		Bounds{Min: Point{-2, -5}, Max: Point{10, 5}},
		driverSensors{f},
		f,
		f,
		f,
		origin,
		slog.New(slog.DiscardHandler),
	)
	if err != nil {
		t.Fatalf("navigation failed at %v, clearance %.3f: %v", f.point, f.minimum, err)
	}
	routeConfig := route.DefaultRouteCheckConfig()
	routeConfig.Flight.MaxDistanceM = 20
	routeConfig.ArrivalDuration = 20 * time.Millisecond
	return d, f, routeConfig
}

func TestDriverDetoursAndConfirmsOriginalGoal(t *testing.T) {
	d, f, config := newTestDriver(t)
	ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
	defer cancel()
	err := d.FollowTarget(
		ctx,
		f.Snapshot(),
		config,
		route.PositionTarget{LatitudeDeg: 8 / coordinates.MetersPerDegree, RelativeAltitudeM: 1.5},
	)
	if err != nil {
		t.Fatalf("follow failed at %v with clearance %.3f: %v", f.point, f.minimum, err)
	}
	if distance(f.point, Point{10, 5}) > config.HorizontalToleranceM || f.minimum < d.config.ClearanceM ||
		f.commands < 10 ||
		math.Hypot(f.north, f.east) >= 0.3 {
		t.Fatalf(
			"unsafe or incomplete flight: position=%v clearance=%v commands=%d velocity=%v/%v",
			f.point,
			f.minimum,
			f.commands,
			f.north,
			f.east,
		)
	}
}

func TestDriverStopsOnLostSensorsAndPoseMismatch(t *testing.T) {
	for _, failure := range []string{"stale", "restart", "pose"} {
		t.Run(failure, func(t *testing.T) {
			d, f, config := newTestDriver(t)
			switch failure {
			case "stale":
				f.stale = true
			case "restart":
				f.restarted = true
			case "pose":
				f.disagreed = true
			}
			err := d.HoldTarget(
				context.Background(),
				f.Snapshot(),
				config,
				route.PositionTarget{RelativeAltitudeM: 1.5},
			)
			if err == nil || f.north != 0 || f.east != 0 || f.commands != 1 {
				t.Fatalf("sensor failure did not stop vehicle: %v", err)
			}
		})
	}
}
