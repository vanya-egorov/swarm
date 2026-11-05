package vehicle

import (
	"context"
	"fmt"
	"time"
)

func watchGroup(
	ctx context.Context, members []FlightMember, origins []State, config FlightCheckConfig, minimumM float64,
) error {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	armed := make([]bool, len(members))
	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			states, err := groupStates(members, config.TelemetryMaxAge)
			if err != nil {
				return err
			}
			for i, state := range states {
				if !sameVehicle(origins[i], state) {
					return ErrSystemIDChanged
				}
				armed[i] = armed[i] || state.Armed
				if armed[i] && (!state.Armed || state.Mode != ModeGuided) {
					return fmt.Errorf("vehicle %d: armed GUIDED state lost", state.SystemID)
				}
				p := state.Position
				if p.RelativeAltitudeM < -0.3 || p.RelativeAltitudeM > config.MaxAltitudeM ||
					horizontalDistance(origins[i].Position.LatitudeDeg, origins[i].Position.LongitudeDeg,
						p.LatitudeDeg, p.LongitudeDeg) > config.MaxDistanceM {
					return fmt.Errorf("vehicle %d: group flight bounds exceeded", state.SystemID)
				}
			}
			if err := checkSeparation(states, minimumM); err != nil {
				return err
			}
		}
	}
}
