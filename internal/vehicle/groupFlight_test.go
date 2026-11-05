package vehicle

import (
	"context"
	"errors"
	"log/slog"
	"sync"
	"testing"
	"time"
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
		s.Position.LatitudeDeg += 20 / metersPerDegree
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
					latitude:        float64(i) * 12 / metersPerDegree,
				}
				fakes[i] = f
				members[i] = FlightMember{f, f, f}
			}
			switch name {
			case "rejected":
				fakes[1].takeoffErr = ErrCommandRejected
			case "drift":
				fakes[1].drift = true
			case "close":
				fakes[1].latitude = 1 / metersPerDegree
			case "duplicate":
				fakes[1].id = 1
			}
			ctx := context.Background()
			if name == "cancelled" {
				var cancel context.CancelFunc
				ctx, cancel = context.WithTimeout(ctx, 40*time.Millisecond)
				defer cancel()
			}
			c := DefaultFlightCheckConfig()
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
			if name == "rejected" && !errors.Is(err, ErrCommandRejected) {
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
