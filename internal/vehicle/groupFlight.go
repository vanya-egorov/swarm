package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"
)

// FlightMember объединяет управление и телеметрию одного участника.
type FlightMember struct {
	Flight FlightController
	Arming ArmingController
	States StateReader
}

// RunGroupFlightCheck выполняет ограниченный взлёт группы и посадку всех при отказе.
// Это проверка стенда: построение формации и маршрутов не выполняется.
func RunGroupFlightCheck(
	ctx context.Context,
	members []FlightMember,
	logger *slog.Logger,
	config FlightCheckConfig,
	minSeparationM float64,
) error {
	if len(members) < 2 || len(members) > 3 || logger == nil || !finite(minSeparationM) || minSeparationM <= 0 {
		return errors.New("invalid flight group")
	}
	if err := config.validate(); err != nil {
		return err
	}
	for _, member := range members {
		if member.Flight == nil || member.Arming == nil || member.States == nil {
			return errors.New("missing flight group dependency")
		}
	}
	readyCtx, cancelReady := context.WithTimeout(ctx, config.ReadyTimeout)
	origins, err := waitForGroup(readyCtx, members, config.TelemetryMaxAge)
	cancelReady()
	if err != nil {
		return err
	}
	for _, state := range origins {
		if state.Armed || state.Mode != ModeStabilize || !onGround(state.Position) {
			return errors.New("group must start disarmed on ground in STABILIZE")
		}
	}
	// Раздельные области полёта не должны пересекаться даже у границ.
	if err := checkSeparation(origins, 2*config.MaxDistanceM+minSeparationM); err != nil {
		return fmt.Errorf("starting positions: %w", err)
	}
	groupCtx, cancel := context.WithCancelCause(ctx)
	defer cancel(nil)
	done := make(chan error, len(members))
	for _, member := range members {
		go func() { done <- RunFlightCheck(groupCtx, member.Flight, member.Arming, member.States, logger, config) }()
	}
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	var result error
	remaining := len(members)
	for remaining > 0 {
		select {
		case err := <-done:
			remaining--
			if err != nil {
				result = errors.Join(result, err)
				cancel(err)
			}
		case <-ticker.C:
			if groupCtx.Err() != nil {
				continue
			}
			states, err := groupStates(members, config.TelemetryMaxAge)
			if err == nil {
				for i, state := range states {
					if !sameVehicle(origins[i], state) {
						err = ErrSystemIDChanged
						break
					}
					if state.Position.RelativeAltitudeM > config.MaxAltitudeM ||
						horizontalDistance(
							origins[i].Position.LatitudeDeg,
							origins[i].Position.LongitudeDeg,
							state.Position.LatitudeDeg,
							state.Position.LongitudeDeg,
						) > config.MaxDistanceM {
						err = errors.New("group flight bounds exceeded")
						break
					}
				}
			}
			if err == nil {
				err = checkSeparation(states, minSeparationM)
			}
			if err != nil {
				logger.Error("group abort", "error", err)
				cancel(err)
			}
		}
	}
	return errors.Join(result, context.Cause(groupCtx))
}

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
