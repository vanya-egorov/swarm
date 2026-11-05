package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
)

// runGroupPhase отменяет соседей при первом отказе, до начала посадки отказавшего аппарата.
func runGroupPhase(ctx context.Context, count int, action func(context.Context, int) error) error {
	phaseCtx, cancel := context.WithCancelCause(ctx)
	defer cancel(nil)
	done := make(chan error, count)
	for i := range count {
		go func() {
			err := action(phaseCtx, i)
			if err != nil {
				err = fmt.Errorf("group member %d: %w", i+1, err)
				cancel(err)
			}
			done <- err
		}()
	}
	var result error
	for range count {
		result = errors.Join(result, <-done)
	}
	return errors.Join(result, context.Cause(phaseCtx))
}

func launchGroup(
	ctx context.Context, members []FlightMember, origins []State, config FlightCheckConfig, logger *slog.Logger,
) error {
	// Каждый аппарат проверяет ожидаемое состояние непосредственно перед следующей командой.
	steps := []func(context.Context, int) error{
		func(ctx context.Context, i int) error { return members[i].Flight.SetMode(ctx, ModeGuided) },
		func(ctx context.Context, i int) error {
			if err := flightCheckState(members[i].States.Snapshot(), origins[i], config, false); err != nil {
				return err
			}
			return members[i].Arming.SetArmed(ctx, true)
		},
		func(ctx context.Context, i int) error {
			if err := flightCheckState(members[i].States.Snapshot(), origins[i], config, true); err != nil {
				return err
			}
			return members[i].Flight.Takeoff(ctx, config.TargetAltitudeM)
		},
	}
	for step, action := range steps {
		if err := runGroupPhase(ctx, len(members), func(ctx context.Context, i int) error {
			return flightCheckCommand(ctx, config.CommandTimeout, func(ctx context.Context) error { return action(ctx, i) })
		}); err != nil {
			return fmt.Errorf("group launch step %d: %w", step+1, err)
		}
	}
	return runGroupPhase(ctx, len(members), func(ctx context.Context, i int) error {
		climbCtx, stop := context.WithTimeout(ctx, config.ClimbTimeout)
		defer stop()
		state, err := watchFlight(climbCtx, members[i].States, origins[i], config,
			func(s State) bool { return s.Position.RelativeAltitudeM >= config.TargetAltitudeM-0.3 })
		if err == nil {
			logger.Info("altitude reached", "systemId", state.SystemID, "altitudeM", state.Position.RelativeAltitudeM)
		}
		return err
	})
}

func landGroup(members []FlightMember, origins []State, config FlightCheckConfig, logger *slog.Logger) error {
	// Отказ одной посадки не отменяет независимые попытки остальных участников.
	done := make(chan error, len(members))
	for i, member := range members {
		go func() {
			ctx, stop := context.WithTimeout(context.Background(), config.LandingTimeout+config.CommandTimeout)
			defer stop()
			err := landAndConfirm(ctx, member.Flight, member.Arming, member.States, origins[i], config, logger)
			if err != nil {
				err = fmt.Errorf("vehicle %d landing: %w", origins[i].SystemID, err)
			}
			done <- err
		}()
	}
	var result error
	for range members {
		result = errors.Join(result, <-done)
	}
	return result
}
