package vehicle

import (
	"context"
	"fmt"
	"time"
)

func waitForGroup(ctx context.Context, members []FlightMember, maxAge time.Duration) ([]State, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if states, err := groupStates(members, maxAge); err == nil {
			return states, nil
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-ticker.C:
		}
	}
}

func groupStates(members []FlightMember, maxAge time.Duration) ([]State, error) {
	states := make([]State, len(members))
	for i, member := range members {
		states[i] = member.States.Snapshot()
		if !freshState(states[i], maxAge) {
			return nil, fmt.Errorf("group member %d: stale telemetry", i+1)
		}
	}
	return states, nil
}

func checkSeparation(states []State, minimumM float64) error {
	for i, a := range states {
		for _, b := range states[i+1:] {
			if a.SystemID == b.SystemID {
				return fmt.Errorf("duplicate vehicle ID %d", a.SystemID)
			}
			distance := horizontalDistance(
				a.Position.LatitudeDeg,
				a.Position.LongitudeDeg,
				b.Position.LatitudeDeg,
				b.Position.LongitudeDeg,
			)
			if distance < minimumM {
				return fmt.Errorf(
					"vehicles %d/%d: separation %.2f m < %.2f m",
					a.SystemID,
					b.SystemID,
					distance,
					minimumM,
				)
			}
		}
	}
	return nil
}
