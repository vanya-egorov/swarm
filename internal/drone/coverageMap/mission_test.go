package coverageMap

import (
	"context"
	"fmt"
	"log/slog"
	"path/filepath"
	"reflect"
	"sync"
	"testing"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordination"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

func missionTestConfig() MissionConfig {
	c := DefaultMissionConfig()
	c.RunID = "coverage-integration"
	c.Area = Area{Max: Point{NorthM: 10, EastM: 18}}
	c.Coverage.SwathWidthM, c.Coverage.LaneSpacingM = 6, 6
	c.Route.Flight.StableDuration, c.Route.Flight.HoldDuration = 0, 0
	c.Route.Flight.ReadyTimeout, c.Route.Flight.ClimbTimeout, c.Route.Flight.LandingTimeout = time.Second, time.Second, time.Second
	c.Route.ArrivalDuration, c.Route.WaypointTimeout = 20*time.Millisecond, time.Second
	c.Network.HeartbeatInterval, c.Network.LeaseTimeout = 10*time.Millisecond, 40*time.Millisecond
	c.Network.ElectionMin, c.Network.ElectionMax, c.Network.PeerTimeout = 100*time.Millisecond, 180*time.Millisecond, 250*time.Millisecond
	c.FailoverTimeout, c.BarrierTimeout = 500*time.Millisecond, 2*time.Second
	for i := range 3 {
		c.Network.Peers = append(
			c.Network.Peers,
			coordination.Peer{SystemID: uint8(i + 1), Address: fmt.Sprintf("127.0.0.1:%d", 18001+i)},
		)
		c.Launches = append(
			c.Launches,
			Launch{SystemID: uint8(i + 1), Position: Point{NorthM: 5, EastM: 3 + 6*float64(i)}},
		)
	}
	return c
}

func TestPrepareMissionIdentityAndValidation(t *testing.T) {
	config := missionTestConfig()
	one, err := PrepareMission(config, 1)
	if err != nil {
		t.Fatal(err)
	}
	two, err := PrepareMission(config, 2)
	if err != nil {
		t.Fatal(err)
	}
	if one.Config.Network.ClusterID != two.Config.Network.ClusterID ||
		one.Config.Network.Stages != two.Config.Network.Stages {
		t.Fatal("agents disagree about mission identity")
	}
	if one.Assignment.SystemID != 1 || two.Assignment.SystemID != 2 ||
		reflect.DeepEqual(one.Route.Waypoints, two.Route.Waypoints) &&
			one.ExpectedLaunch == two.ExpectedLaunch {
		t.Fatal("wrong local assignment")
	}
	config.RunID = "different-run"
	different, err := PrepareMission(config, 1)
	if err != nil || different.Config.Network.ClusterID == one.Config.Network.ClusterID {
		t.Fatal("run ID missing from fingerprint")
	}
	for _, name := range []string{"launch", "duplicate", "missing", "swath", "altitude", "limit", "timing", "member"} {
		t.Run(name, func(t *testing.T) {
			c := missionTestConfig()
			switch name {
			case "launch":
				c.Launches[0].Position.EastM = 7
			case "duplicate":
				c.Launches[1].SystemID = 1
			case "missing":
				c.Launches = c.Launches[:2]
			case "swath":
				c.Coverage.SwathWidthM, c.Coverage.LaneSpacingM = 1, 1
			case "altitude":
				c.Coverage.AltitudeM = 5
			case "limit":
				c.Route.Flight.MaxDistanceM = 1
			case "timing":
				c.FailoverTimeout = time.Millisecond
			case "member":
				c.Network.Peers[1].SystemID = 1
			}
			if _, err := PrepareMission(c, 1); err == nil {
				t.Fatal("invalid mission accepted")
			}
		})
	}
}

type missionBus struct {
	mu       sync.Mutex
	packets  map[uint8]chan coordination.Message
	isolated map[uint8]bool
	leader   uint8
}

type missionTransport struct {
	bus *missionBus
	id  uint8
}

func (t missionTransport) Send(ctx context.Context, id uint8, m coordination.Message) error {
	t.bus.mu.Lock()
	drop := t.bus.isolated[t.id] || t.bus.isolated[id]
	if m.Kind == "heartbeat" && m.Release > 0 && !drop {
		t.bus.leader = m.From
	}
	ch := t.bus.packets[id]
	t.bus.mu.Unlock()
	if drop {
		return nil
	}
	select {
	case ch <- m:
		return nil
	case <-ctx.Done():
		return ctx.Err()
	default:
		return nil
	}
}
func (t missionTransport) Receive(ctx context.Context) (coordination.Message, error) {
	select {
	case m := <-t.bus.packets[t.id]:
		return m, nil
	case <-ctx.Done():
		return coordination.Message{}, ctx.Err()
	}
}
func (t missionTransport) Close() error { return nil }

type missionFake struct {
	mu       sync.Mutex
	id       uint8
	armed    bool
	mode     telemetry.FlightMode
	position route.PositionTarget
	targets  []route.PositionTarget
	landed   bool
	bad      bool
	blocked  bool
}

func (f *missionFake) Snapshot() telemetry.State {
	f.mu.Lock()
	defer f.mu.Unlock()
	now := time.Now()
	stamp := now
	if f.bad {
		stamp = now.Add(-time.Hour)
	}
	return telemetry.State{SystemID: f.id, ComponentID: 1, Armed: f.armed, Mode: f.mode, LastHeartbeat: stamp,
		Position: &telemetry.Position{SystemID: f.id, ComponentID: 1, LatitudeDeg: f.position.LatitudeDeg,
			LongitudeDeg: f.position.LongitudeDeg, RelativeAltitudeM: f.position.RelativeAltitudeM, ReceivedAt: stamp},
		ServoOutputs: &telemetry.ServoOutputs{SystemID: f.id, ComponentID: 1, ReceivedAt: stamp}}
}
func (f *missionFake) SetMode(_ context.Context, mode telemetry.FlightMode) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.mode = mode
	return nil
}
func (f *missionFake) SetArmed(_ context.Context, armed bool) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.armed = armed
	return nil
}
func (f *missionFake) Takeoff(_ context.Context, altitude float64) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.position.RelativeAltitudeM = altitude
	return nil
}
func (f *missionFake) Land(context.Context) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.mode, f.armed, f.landed = telemetry.ModeLand, false, true
	f.position.RelativeAltitudeM = 0
	return nil
}
func (f *missionFake) SetPositionTarget(_ context.Context, p route.PositionTarget) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.targets) == 0 || f.targets[len(f.targets)-1] != p {
		f.targets = append(f.targets, p)
	}
	if !f.blocked {
		f.position = p
	}
	return nil
}

func TestDistributedCoverageMission(t *testing.T) {
	for _, failure := range []string{"none", "leader-link", "leader-health", "leader-stuck"} {
		t.Run(failure, func(t *testing.T) {
			config := missionTestConfig()
			bus := &missionBus{packets: map[uint8]chan coordination.Message{}, isolated: map[uint8]bool{}}
			for id := uint8(1); id <= 3; id++ {
				bus.packets[id] = make(chan coordination.Message, 1024)
			}
			ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
			defer cancel()
			fakes := map[uint8]*missionFake{}
			done := make(chan struct {
				id  uint8
				err error
			}, 3)
			for id := uint8(1); id <= 3; id++ {
				prepared, err := PrepareMission(config, id)
				if err != nil {
					t.Fatal(err)
				}
				p := prepared.ExpectedLaunch
				p.RelativeAltitudeM = 0
				f := &missionFake{id: id, mode: telemetry.ModeStabilize, position: p}
				fakes[id] = f
				base := filepath.Join(t.TempDir(), "agent")
				runner := MissionRunner{Config: config, SystemID: id, Transport: missionTransport{bus, id},
					Store: coordination.FileStore{Path: base + ".json"}, StartMarker: base + ".flight",
					Vehicle: Vehicle{
						Flight:    f,
						Arming:    f,
						Positions: f,
						States:    f,
					}, Logger: slog.New(slog.DiscardHandler)}
				go func() {
					done <- struct {
						id  uint8
						err error
					}{id, runner.Run(ctx)}
				}()
			}
			failed := uint8(0)
			if failure != "none" {
				for failed == 0 {
					bus.mu.Lock()
					leader := bus.leader
					bus.mu.Unlock()
					if leader != 0 {
						f := fakes[leader]
						f.mu.Lock()
						if len(f.targets) >= 2 && !f.landed {
							failed = leader
							switch failure {
							case "leader-health":
								f.bad = true
							case "leader-stuck":
								f.blocked = true
							}
						}
						f.mu.Unlock()
					}
					if ctx.Err() != nil {
						t.Fatal("leader did not start route")
					}
					if failed == 0 {
						time.Sleep(5 * time.Millisecond)
					}
				}
				if failure == "leader-link" {
					bus.mu.Lock()
					bus.isolated[failed] = true
					bus.mu.Unlock()
				}
			}
			for range 3 {
				select {
				case result := <-done:
					if failure == "none" && result.err != nil {
						t.Errorf("vehicle %d: %v", result.id, result.err)
					}
					if failure != "none" && result.err == nil {
						t.Errorf("vehicle %d reported full coverage with failed member", result.id)
					}
					f := fakes[result.id]
					f.mu.Lock()
					if !f.landed || f.armed {
						t.Errorf("vehicle %d returned without landing", result.id)
					}
					if result.id != failed && len(f.targets) != 5 {
						t.Errorf(
							"vehicle %d route repeated or incomplete: %d targets (%v)",
							result.id,
							len(f.targets),
							result.err,
						)
					}
					f.mu.Unlock()
				case <-ctx.Done():
					t.Fatal("distributed mission did not terminate")
				}
			}
		})
	}
}

func TestMissionStartMarkerPreventsSecondFlight(t *testing.T) {
	config := missionTestConfig()
	prepared, err := PrepareMission(config, 1)
	if err != nil {
		t.Fatal(err)
	}
	base := filepath.Join(t.TempDir(), "flight")
	s := &missionSession{mission: prepared, marker: base}
	// Ошибка публикации здесь несущественна: проверяется эксклюзивная запись до команд.
	if err := s.createMarker(); err != nil {
		t.Fatal(err)
	}
	if err := s.createMarker(); err == nil {
		t.Fatal("repeated automatic flight allowed")
	}
}

func TestMarginPreservesCoverage(t *testing.T) {
	config := Config{SwathWidthM: 6, LaneSpacingM: 2, AltitudeM: 1.5, BoundaryMarginM: 2.4}
	area := Area{Max: Point{NorthM: 10, EastM: 6}}
	points, err := GenerateRoute(area, config)
	if err != nil {
		t.Fatal(err)
	}
	checkCoverage(t, Assignment{Region: Region{Area: area}, Waypoints: points}, config)
	for _, p := range points {
		if !inside(Point{p.NorthM, p.EastM}, area, 2.4) {
			t.Fatal("margin not preserved")
		}
	}
	config.BoundaryMarginM = 4
	if _, err := GenerateRoute(area, config); err == nil {
		t.Fatal("uncoverable margin allowed")
	}
	config.BoundaryMarginM = 2.4
	if _, err := GenerateRoute(Area{Max: Point{NorthM: 3, EastM: 3}}, config); err == nil {
		t.Fatal("narrow region allowed")
	}
}
