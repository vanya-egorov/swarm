package group

import (
	"context"
	"errors"
	"log/slog"
	"sync"
	"testing"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type groupFake struct {
	mu sync.Mutex
	flightCheckFake
	id       uint8
	latitude float64
	drift    bool
}

func (f *groupFake) Snapshot() State {
	f.mu.Lock()
	defer f.mu.Unlock()
	s := f.flightCheckFake.Snapshot()
	s.SystemID = f.id
	s.Position.SystemID = f.id
	s.ServoOutputs.SystemID = f.id
	s.Position.LatitudeDeg = f.latitude
	if f.drift && f.armed {
		s.Position.LatitudeDeg += 20 / coordinates.MetersPerDegree
	}
	return s
}
func (f *groupFake) SetMode(ctx context.Context, mode FlightMode) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.SetMode(ctx, mode)
}
func (f *groupFake) SetArmed(ctx context.Context, armed bool) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.SetArmed(ctx, armed)
}
func (f *groupFake) Takeoff(ctx context.Context, altitude float64) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.Takeoff(ctx, altitude)
}
func (f *groupFake) Land(ctx context.Context) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.Land(ctx)
}
func TestGroupFlightCheck(t *testing.T) {
	for _, name := range []string{"success", "rejected", "drift", "close", "duplicate", "cancelled"} {
		t.Run(name, func(t *testing.T) {
			fakes := make([]*groupFake, 3)
			members := make([]FlightMember, 3)
			for i := range members {
				f := &groupFake{
					flightCheckFake: flightCheckFake{mode: ModeStabilize},
					id:              uint8(i + 1),
					latitude:        float64(i) * 12 / coordinates.MetersPerDegree,
				}
				fakes[i] = f
				members[i] = FlightMember{f, f, f}
			}
			switch name {
			case "rejected":
				fakes[1].takeoffErr = flight.ErrCommandRejected
			case "drift":
				fakes[1].drift = true
			case "close":
				fakes[1].latitude = 1 / coordinates.MetersPerDegree
			case "duplicate":
				fakes[1].id = 1
			}
			ctx := context.Background()
			if name == "cancelled" {
				var cancel context.CancelFunc
				ctx, cancel = context.WithTimeout(ctx, 40*time.Millisecond)
				defer cancel()
			}
			c := flight.DefaultFlightCheckConfig()
			c.StableDuration = 0
			c.HoldDuration = 100 * time.Millisecond
			c.ReadyTimeout = time.Second
			c.CommandTimeout = time.Second
			c.LandingTimeout = time.Second
			c.MaxDistanceM = 2
			err := RunGroupFlightCheck(ctx, members, slog.New(slog.DiscardHandler), c, 4)
			if (err == nil) != (name == "success") {
				t.Fatalf("unexpected result: %v", err)
			}
			if name == "rejected" && !errors.Is(err, flight.ErrCommandRejected) {
				t.Fatalf("rejection lost: %v", err)
			}
			if name == "cancelled" && !errors.Is(err, context.DeadlineExceeded) {
				t.Fatalf("cancellation lost: %v", err)
			}
			for _, f := range fakes {
				if f.armed || f.altitude != 0 {
					t.Fatalf("group returned before cleanup: %+v", f.Snapshot())
				}
				if (name == "close" || name == "duplicate") && len(f.calls) > 0 {
					t.Fatal("commands sent before preflight validation")
				}
			}
		})
	}
}

// groupRouteFake разделяет принятую цель и измеренную позицию для проверки барьеров.
type groupRouteFake struct {
	mu sync.Mutex
	flightCheckFake
	id                   uint8
	originLat, originLon float64
	targets              []PositionTarget
	position             PositionTarget
	targetErr            error
	blocked              bool
	stale                bool
	freeze               bool
	frozenAt             time.Time
	requested            chan struct{}
	landRequested        chan struct{}
	landRelease          <-chan struct{}
}

func newGroupRouteFakes() ([]*groupRouteFake, []RouteMember) {
	fakes := make([]*groupRouteFake, 3)
	members := make([]RouteMember, 3)
	for i := range members {
		f := &groupRouteFake{flightCheckFake: flightCheckFake{mode: ModeStabilize}, id: uint8(i + 1),
			originLat: float64(
				i,
			) * 12 / coordinates.MetersPerDegree, requested: make(chan struct{}, 1), landRequested: make(chan struct{}, 1)}
		fakes[i] = f
		members[i] = RouteMember{FlightMember: FlightMember{Flight: f, Arming: f, States: f}, Positions: f}
	}
	return fakes, members
}

func (f *groupRouteFake) Snapshot() State {
	f.mu.Lock()
	defer f.mu.Unlock()
	s := f.flightCheckFake.Snapshot()
	s.SystemID, s.Position.SystemID, s.ServoOutputs.SystemID = f.id, f.id, f.id
	s.Position.LatitudeDeg, s.Position.LongitudeDeg = f.originLat, f.originLon
	if len(f.targets) > 0 {
		s.Position.LatitudeDeg, s.Position.LongitudeDeg = f.position.LatitudeDeg, f.position.LongitudeDeg
		if f.stale && f.armed {
			s.Position.ReceivedAt = time.Now().Add(-time.Hour)
		}
		if f.freeze && f.armed {
			s.Position.ReceivedAt = f.frozenAt
		}
	}
	return s
}
func (f *groupRouteFake) SetMode(ctx context.Context, mode FlightMode) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.SetMode(ctx, mode)
}
func (f *groupRouteFake) SetArmed(ctx context.Context, armed bool) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.SetArmed(ctx, armed)
}
func (f *groupRouteFake) Takeoff(ctx context.Context, altitude float64) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.Takeoff(ctx, altitude)
}
func (f *groupRouteFake) Land(ctx context.Context) error {
	select {
	case f.landRequested <- struct{}{}:
	default:
	}
	if f.landRelease != nil {
		select {
		case <-f.landRelease:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.flightCheckFake.Land(ctx)
}
func (f *groupRouteFake) SetPositionTarget(_ context.Context, target PositionTarget) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.targets) == 0 || target != f.targets[len(f.targets)-1] {
		f.targets = append(f.targets, target)
	}
	if len(f.targets) > 1 {
		select {
		case f.requested <- struct{}{}:
		default:
		}
		if f.targetErr != nil {
			return f.targetErr
		}
	}
	if !f.blocked || len(f.targets) == 1 {
		f.position, f.altitude = target, target.RelativeAltitudeM
	}
	if f.frozenAt.IsZero() {
		f.frozenAt = time.Now()
	}
	return nil
}
func (f *groupRouteFake) releaseTarget() {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.blocked = false
	f.position = f.targets[len(f.targets)-1]
	f.altitude = f.position.RelativeAltitudeM
}

func groupRouteTestConfig() RouteCheckConfig {
	config := route.DefaultRouteCheckConfig()
	config.Flight.StableDuration = 0
	config.Flight.HoldDuration = 0
	config.ArrivalDuration = 20 * time.Millisecond
	config.Flight.MaxDistanceM = 3.5
	config.Flight.ClimbTimeout = 200 * time.Millisecond
	config.WaypointTimeout = 300 * time.Millisecond
	return config
}

func TestGroupRouteFailuresLandEveryone(t *testing.T) {
	for _, name := range []string{"success", "command", "stuck", "stale", "frozen", "cancel"} {
		t.Run(name, func(t *testing.T) {
			fakes, members := newGroupRouteFakes()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch name {
			case "command":
				fakes[1].targetErr = flight.ErrCommandRejected
			case "stuck":
				fakes[1].blocked = true
			case "stale":
				fakes[1].stale = true
			case "frozen":
				fakes[1].freeze = true
			case "cancel":
				go func() {
					select {
					case <-fakes[1].requested:
						cancel()
					case <-ctx.Done():
					}
				}()
			}
			config := groupRouteTestConfig()
			err := RunGroupRouteCheck(ctx, members, slog.New(slog.DiscardHandler), config, 4)
			if (err == nil) != (name == "success") {
				t.Fatalf("%s: %v", name, err)
			}
			if name == "command" && !errors.Is(err, flight.ErrCommandRejected) {
				t.Fatalf("rejection lost: %v", err)
			}
			if (name == "stuck" || name == "frozen") && !errors.Is(err, context.DeadlineExceeded) {
				t.Fatalf("timeout lost: %v", err)
			}
			if name == "cancel" && !errors.Is(err, context.Canceled) {
				t.Fatalf("cancel lost: %v", err)
			}
			for _, f := range fakes {
				s := f.Snapshot()
				if s.Armed || s.Position.RelativeAltitudeM != 0 {
					t.Fatalf("vehicle %d not landed", f.id)
				}
				if name == "success" && len(f.targets) != len(config.Waypoints)+1 {
					t.Fatalf("vehicle %d targets: %v", f.id, f.targets)
				}
			}
		})
	}
}

func TestGroupRouteWaitsForSlowMember(t *testing.T) {
	fakes, members := newGroupRouteFakes()
	fakes[2].blocked = true
	config := groupRouteTestConfig()
	config.WaypointTimeout = time.Second
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- RunGroupRouteCheck(ctx, members, slog.New(slog.DiscardHandler), config, 4) }()
	select {
	case <-fakes[2].requested:
	case <-ctx.Done():
		t.Fatal("first waypoint not started")
	}
	// Даём быстрым участникам выдержать допуск прибытия и ещё несколько циклов координатора.
	time.Sleep(3 * config.ArrivalDuration)
	for _, f := range fakes {
		f.mu.Lock()
		count := len(f.targets)
		f.mu.Unlock()
		if count != 2 {
			t.Fatalf("vehicle %d advanced before slow member: %d targets", f.id, count)
		}
	}
	fakes[2].releaseTarget()
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}

func TestGroupFailureStartsOtherLandingsBeforeFailedLandingCompletes(t *testing.T) {
	fakes, members := newGroupRouteFakes()
	fakes[1].targetErr = flight.ErrCommandRejected
	release := make(chan struct{})
	fakes[1].landRelease = release
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	done := make(chan error, 1)
	go func() {
		done <- RunGroupRouteCheck(ctx, members, slog.New(slog.DiscardHandler), groupRouteTestConfig(), 4)
	}()
	for _, f := range fakes {
		select {
		case <-f.landRequested:
		case <-ctx.Done():
			t.Errorf("vehicle %d landing delayed", f.id)
		}
	}
	select {
	case err := <-done:
		t.Errorf("returned before all landings: %v", err)
	default:
	}
	close(release)
	if err := <-done; !errors.Is(err, flight.ErrCommandRejected) {
		t.Fatalf("rejection lost: %v", err)
	}
}

func TestGroupRouteValidatesEveryoneBeforeCommands(t *testing.T) {
	for _, name := range []string{"route", "position", "polar", "duplicate", "close", "missing"} {
		t.Run(name, func(t *testing.T) {
			fakes, members := newGroupRouteFakes()
			config := groupRouteTestConfig()
			switch name {
			case "route":
				config.Waypoints[0].NorthM = 10
			case "position":
				members[2].Positions = nil
			case "polar":
				fakes[2].originLat = 89
			case "duplicate":
				fakes[2].id = 1
			case "close":
				fakes[2].originLat = 0
			case "missing":
				members[2].States = nil
			}
			err := RunGroupRouteCheck(context.Background(), members, slog.New(slog.DiscardHandler), config, 4)
			if err == nil {
				t.Fatal("invalid route accepted")
			}
			for _, f := range fakes {
				if len(f.calls) != 0 {
					t.Fatalf("commands before validation: %v", f.calls)
				}
			}
		})
	}
}

type flightCheckFake struct {
	armed      bool
	mode       FlightMode
	altitude   float64
	calls      []string
	takeoffErr error
}

func (f *flightCheckFake) Snapshot() State {
	now := time.Now()
	return State{SystemID: 1, ComponentID: 1, Armed: f.armed, Mode: f.mode, LastHeartbeat: now,
		Position:     &Position{SystemID: 1, ComponentID: 1, RelativeAltitudeM: f.altitude, ReceivedAt: now},
		ServoOutputs: &telemetry.ServoOutputs{SystemID: 1, ComponentID: 1, PWMUS: [16]uint16{1000, 1000, 1000, 1000}, ReceivedAt: now}}
}

func (f *flightCheckFake) SetMode(_ context.Context, mode FlightMode) error {
	f.calls = append(f.calls, "mode")
	f.mode = mode
	return nil
}

func (f *flightCheckFake) SetArmed(_ context.Context, armed bool) error {
	if armed {
		f.calls = append(f.calls, "arm")
	} else {
		f.calls = append(f.calls, "disarm")
	}
	f.armed = armed
	return nil
}

func (f *flightCheckFake) Takeoff(_ context.Context, altitude float64) error {
	f.calls = append(f.calls, "takeoff")
	if f.takeoffErr == nil {
		f.altitude = altitude
	}
	return f.takeoffErr
}

func (f *flightCheckFake) Land(context.Context) error {
	f.calls = append(f.calls, "land")
	f.mode, f.altitude, f.armed = ModeLand, 0, false
	return nil
}
