package route

import (
	"context"
	"errors"
	"log/slog"
	"math"
	"reflect"
	"testing"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type routeFake struct {
	armed                 bool
	mode                  FlightMode
	altitude              float64
	calls                 []string
	target                PositionTarget
	sendErr               error
	stuck, escaped, stale bool
	climbing              bool
	cancel                context.CancelFunc
}

func (f *routeFake) Snapshot() State {
	now := time.Now()
	position := &Position{
		SystemID: 1, ComponentID: 1,
		LatitudeDeg: f.target.LatitudeDeg, LongitudeDeg: f.target.LongitudeDeg,
		RelativeAltitudeM: f.altitude, ReceivedAt: now,
	}
	if f.climbing && f.armed {
		position.DownVelocityMS = -1
	}
	if len(f.calls) > 0 && f.calls[len(f.calls)-1] == "target" {
		if f.escaped {
			position.LatitudeDeg = 1
		}
		if f.stale {
			position.ReceivedAt = time.Now().Add(-time.Hour)
		}
	}
	outputs := &telemetry.ServoOutputs{SystemID: 1, ComponentID: 1, ReceivedAt: now}
	return State{SystemID: 1, ComponentID: 1, Armed: f.armed, Mode: f.mode,
		LastHeartbeat: now, Position: position, ServoOutputs: outputs}
}

func (f *routeFake) SetMode(_ context.Context, mode FlightMode) error {
	f.calls = append(f.calls, "mode")
	f.mode = mode
	return nil
}

func (f *routeFake) SetArmed(_ context.Context, armed bool) error {
	if armed {
		f.calls = append(f.calls, "arm")
	} else {
		f.calls = append(f.calls, "disarm")
	}
	f.armed = armed
	return nil
}

func (f *routeFake) Takeoff(_ context.Context, altitude float64) error {
	f.calls = append(f.calls, "takeoff")
	f.altitude = altitude
	return nil
}

func (f *routeFake) Land(context.Context) error {
	f.calls = append(f.calls, "land")
	f.mode, f.altitude, f.armed = ModeLand, 0, false
	return nil
}

func (f *routeFake) SetPositionTarget(_ context.Context, target PositionTarget) error {
	f.calls = append(f.calls, "target")
	if f.cancel != nil {
		f.cancel()
	}
	if !f.stuck {
		f.target = target
		f.altitude = target.RelativeAltitudeM
	}
	return f.sendErr
}
func routeTestConfig() RouteCheckConfig {
	c := DefaultRouteCheckConfig()
	c.Flight.StableDuration = 0
	c.Flight.HoldDuration = 0
	c.ArrivalDuration = 20 * time.Millisecond
	c.WaypointTimeout = 150 * time.Millisecond
	c.Waypoints = c.Waypoints[:2]
	return c
}
func TestRouteCheck(t *testing.T) {
	for _, name := range []string{"success", "rejected", "unreachable", "bounds", "stale", "cancel"} {
		t.Run(name, func(t *testing.T) {
			f := &routeFake{mode: ModeStabilize}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch name {
			case "rejected":
				f.sendErr = flight.ErrCommandRejected
			case "unreachable":
				f.stuck = true
			case "bounds":
				f.escaped = true
			case "stale":
				f.stale = true
			case "cancel":
				f.cancel = cancel
			}
			err := RunRouteCheck(ctx, f, f, f, f, slog.New(slog.DiscardHandler), routeTestConfig())
			if (err == nil) != (name == "success") {
				t.Fatalf("unexpected result: %v", err)
			}
			if name == "cancel" && !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation lost: %v", err)
			}
			if name == "unreachable" && !errors.Is(err, context.DeadlineExceeded) {
				t.Fatalf("timeout lost: %v", err)
			}
			if name == "rejected" && !errors.Is(err, flight.ErrCommandRejected) {
				t.Fatalf("send error lost: %v", err)
			}
			if f.armed || f.calls[len(f.calls)-1] != "land" {
				t.Fatalf("landing missing: %v", f.calls)
			}
			if name == "success" &&
				!reflect.DeepEqual(f.calls, []string{"mode", "arm", "takeoff", "target", "target", "target", "land"}) {
				t.Fatal(f.calls)
			}
		})
	}
}
func TestRouteValidationBeforeCommands(t *testing.T) {
	for _, p := range []Waypoint{{6, 0, 1.5}, {0, 0, 3}, {math.NaN(), 0, 1.5}, {0, math.Inf(1), 1.5}} {
		f := &routeFake{mode: ModeStabilize}
		c := routeTestConfig()
		c.Waypoints = []Waypoint{p}
		if err := RunRouteCheck(context.Background(), f, f, f, f, slog.Default(), c); err == nil || len(f.calls) > 0 {
			t.Fatalf("invalid route triggered commands: %v %v", err, f.calls)
		}
	}
}
func TestRouteCoordinates(t *testing.T) {
	origin := &Position{LatitudeDeg: 55, LongitudeDeg: 37}
	points, err := TargetsFrom(origin, []Waypoint{{2, 3, 1.5}, {0, 0, 1.5}})
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs((points[0].LatitudeDeg-55)*coordinates.MetersPerDegree-2) > 1e-6 ||
		math.Abs((points[0].LongitudeDeg-37)*coordinates.MetersPerDegree*math.Cos(55*math.Pi/180)-3) > 1e-6 {
		t.Fatal(points)
	}
	if points[1].LatitudeDeg != 55 || points[1].LongitudeDeg != 37 {
		t.Fatal(points)
	}
	// Короткий путь через 180-й меридиан не должен выглядеть как выход за границы.
	if d := coordinates.HorizontalDistance(0, 179.99999, 0, -179.99999); d > 3 {
		t.Fatal(d)
	}
	if _, err := TargetsFrom(&Position{LatitudeDeg: 89}, []Waypoint{{1, 0, 1.5}}); err == nil {
		t.Fatal("polar origin accepted")
	}
}
func TestFlightRejectsInvalidPosition(t *testing.T) {
	f := &routeFake{armed: true, mode: ModeGuided, altitude: 1.5}
	before := f.Snapshot()
	s := f.Snapshot()
	s.Position.LatitudeDeg = math.NaN()
	if flight.FreshState(s, time.Second) || flight.CheckState(s, before, flight.DefaultFlightCheckConfig(), true) == nil {
		t.Fatal("NaN bypassed flight bounds")
	}
}

func TestRouteWaitsForHoverBeforeHorizontalMovement(t *testing.T) {
	f := &routeFake{mode: ModeStabilize, climbing: true}
	config := routeTestConfig()
	config.Flight.ClimbTimeout = 100 * time.Millisecond
	err := RunRouteCheck(context.Background(), f, f, f, f, slog.New(slog.DiscardHandler), config)
	if !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("unstable hover accepted: %v", err)
	}
	if f.target.LatitudeDeg != 0 || f.target.LongitudeDeg != 0 {
		t.Fatal("horizontal target sent before stable hover")
	}
	if f.armed || f.calls[len(f.calls)-1] != "land" {
		t.Fatal("landing missing after hover timeout")
	}
}
