package vehicle

import (
	"context"
	"fmt"
	"time"
)

// followGroupTargets продолжает удерживать цели быстрых участников до готовности всей группы.
func followGroupTargets(
	ctx context.Context, members []RouteMember, origins []State, config RouteCheckConfig, targets []PositionTarget,
) error {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	firstSent := make([]time.Time, len(members))
	arrived := make([]time.Time, len(members))
	var lastSent time.Time
	for {
		if err := ctx.Err(); err != nil {
			return context.Cause(ctx)
		}
		allArrived := true
		resend := lastSent.IsZero() || time.Since(lastSent) >= time.Second
		for i, member := range members {
			state := member.States.Snapshot()
			if err := flightCheckState(state, origins[i], config.Flight, true); err != nil {
				return fmt.Errorf("vehicle %d target: %w", origins[i].SystemID, err)
			}
			if resend {
				sent := time.Now()
				if err := flightCheckCommand(ctx, config.Flight.CommandTimeout, func(ctx context.Context) error {
					return member.Positions.SetPositionTarget(ctx, targets[i])
				}); err != nil {
					return fmt.Errorf("vehicle %d target command: %w", origins[i].SystemID, err)
				}
				if firstSent[i].IsZero() {
					firstSent[i] = sent
				}
			}
			p := state.Position
			if p.ReceivedAt.After(firstSent[i]) && nearTarget(p, targets[i], config) {
				if arrived[i].IsZero() {
					arrived[i] = p.ReceivedAt
				}
			} else {
				arrived[i] = time.Time{}
			}
			if arrived[i].IsZero() || p.ReceivedAt.Sub(arrived[i]) < config.ArrivalDuration {
				allArrived = false
			}
		}
		if resend {
			lastSent = time.Now()
		}
		if allArrived {
			return nil
		}
		select {
		case <-ctx.Done():
			return context.Cause(ctx)
		case <-ticker.C:
		}
	}
}
