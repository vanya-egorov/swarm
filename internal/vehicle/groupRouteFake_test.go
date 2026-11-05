package vehicle

import (
	"context"
	"sync"
	"time"
)

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
			) * 12 / metersPerDegree, requested: make(chan struct{}, 1), landRequested: make(chan struct{}, 1)}
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
