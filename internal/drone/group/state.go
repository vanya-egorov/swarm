package group

import (
	"context"
	"fmt"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

// FlightMember combines a drone's command controllers and state reader.
type FlightMember struct {
	Flight flight.FlightController
	Arming flight.ArmingController
	States telemetry.StateReader
}

// RouteMember adds a position controller to a flight group member.
type RouteMember struct {
	FlightMember
	Positions route.PositionController
}

type State = telemetry.State
type Position = telemetry.Position
type FlightMode = telemetry.FlightMode
type FlightCheckConfig = flight.FlightCheckConfig
type RouteCheckConfig = route.RouteCheckConfig
type PositionTarget = route.PositionTarget

const (
	ModeStabilize = telemetry.ModeStabilize
	ModeGuided    = telemetry.ModeGuided
	ModeLand      = telemetry.ModeLand
)

func waitForGroup(ctx context.Context, members []FlightMember, maxAge time.Duration) ([]State, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	states := make([]State, len(members))
	for {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if err := readGroupStates(members, maxAge, states); err == nil {
			return states, nil
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-ticker.C:
		}
	}
}

func readGroupStates(members []FlightMember, maxAge time.Duration, states []State) error {
	for i, member := range members {
		states[i] = member.States.Snapshot()
		if !flight.FreshState(states[i], maxAge) {
			return fmt.Errorf("group member %d: stale telemetry", i+1)
		}
	}
	return nil
}

func checkSeparation(states []State, minimumM float64) error {
	for i, a := range states {
		for _, b := range states[i+1:] {
			if a.SystemID == b.SystemID {
				return fmt.Errorf("duplicate vehicle ID %d", a.SystemID)
			}
			distance := coordinates.HorizontalDistance(
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

func watchGroup(
	ctx context.Context, members []FlightMember, origins []State, config FlightCheckConfig, minimumM float64,
) error {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	armed := make([]bool, len(members))
	states := make([]State, len(members))
	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			if err := readGroupStates(members, config.TelemetryMaxAge, states); err != nil {
				return err
			}
			for i, state := range states {
				if !flight.SameDrone(origins[i], state) {
					return telemetry.ErrSystemIDChanged
				}
				armed[i] = armed[i] || state.Armed
				if armed[i] && (!state.Armed || state.Mode != ModeGuided) {
					return fmt.Errorf("vehicle %d: armed GUIDED state lost", state.SystemID)
				}
				p := state.Position
				if p.RelativeAltitudeM < -0.3 || p.RelativeAltitudeM > config.MaxAltitudeM ||
					coordinates.HorizontalDistance(origins[i].Position.LatitudeDeg, origins[i].Position.LongitudeDeg,
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
