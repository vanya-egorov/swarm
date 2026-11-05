package vehicle

import (
	"context"
	"errors"
	"log/slog"
	"testing"
	"time"
)

func groupRouteTestConfig() RouteCheckConfig {
	config := routeTestConfig()
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
				fakes[1].targetErr = ErrCommandRejected
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
			err := RunGroupRouteCheck(ctx, members, slog.New(slog.DiscardHandler), groupRouteTestConfig(), 4)
			if (err == nil) != (name == "success") {
				t.Fatalf("%s: %v", name, err)
			}
			if name == "command" && !errors.Is(err, ErrCommandRejected) {
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
				if name == "success" && len(f.targets) != 3 {
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
	fakes[1].targetErr = ErrCommandRejected
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
	if err := <-done; !errors.Is(err, ErrCommandRejected) {
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
