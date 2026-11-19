package group

import (
	"context"
	"errors"
	"fmt"
	"log/slog"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
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
			if err := flight.CheckState(members[i].States.Snapshot(), origins[i], config, false); err != nil {
				return err
			}
			return members[i].Arming.SetArmed(ctx, true)
		},
		func(ctx context.Context, i int) error {
			if err := flight.CheckState(members[i].States.Snapshot(), origins[i], config, true); err != nil {
				return err
			}
			return members[i].Flight.Takeoff(ctx, config.TargetAltitudeM)
		},
	}
	for step, action := range steps {
		if err := runGroupPhase(ctx, len(members), func(ctx context.Context, i int) error {
			return flight.RunCommand(ctx, config.CommandTimeout, func(ctx context.Context) error { return action(ctx, i) })
		}); err != nil {
			return fmt.Errorf("group launch step %d: %w", step+1, err)
		}
	}
	return runGroupPhase(ctx, len(members), func(ctx context.Context, i int) error {
		climbCtx, stop := context.WithTimeout(ctx, config.ClimbTimeout)
		defer stop()
		state, err := flight.WatchFlight(climbCtx, members[i].States, origins[i], config,
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
			err := flight.LandAndConfirm(ctx, member.Flight, member.Arming, member.States, origins[i], config, logger)
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

// RunGroupFlightCheck выполняет общий взлёт, удержание и посадку всех при отказе.
func RunGroupFlightCheck(
	ctx context.Context, members []FlightMember, logger *slog.Logger,
	config FlightCheckConfig, minSeparationM float64,
) error {
	return runGroupFlight(ctx, members, logger, config, minSeparationM, nil)
}

// runGroupFlight ждёт завершения каждого этапа всей группой и всегда дожидается посадок.
func runGroupFlight(
	ctx context.Context, members []FlightMember, logger *slog.Logger,
	config FlightCheckConfig, minSeparationM float64,
	task func(context.Context, []State) error,
) (result error) {
	origins, err := prepareGroup(ctx, members, logger, config, minSeparationM)
	if err != nil {
		return err
	}
	groupCtx, cancel := context.WithCancelCause(ctx)
	guardDone := make(chan error, 1)
	go func() {
		err := watchGroup(groupCtx, members, origins, config, minSeparationM)
		if err != nil {
			logger.Error("group abort", "error", err)
			cancel(err)
		}
		guardDone <- err
	}()
	defer func() {
		// Посадка имеет собственные контексты и не отменяется вместе с полётным заданием.
		cause := context.Cause(groupCtx)
		cancel(nil)
		guardErr := <-guardDone
		result = errors.Join(result, cause, guardErr, landGroup(members, origins, config, logger))
	}()
	if err := launchGroup(groupCtx, members, origins, config, logger); err != nil {
		return err
	}
	if task != nil {
		if err := task(groupCtx, origins); err != nil {
			return err
		}
	}
	return runGroupPhase(groupCtx, len(members), func(ctx context.Context, i int) error {
		holdCtx, stop := context.WithTimeout(ctx, config.HoldDuration)
		defer stop()
		_, err := flight.WatchFlight(holdCtx, members[i].States, origins[i], config, func(State) bool { return false })
		if ctx.Err() != nil {
			return context.Cause(ctx)
		}
		if errors.Is(err, context.DeadlineExceeded) {
			return nil
		}
		return err
	})
}

func prepareGroup(
	ctx context.Context, members []FlightMember, logger *slog.Logger,
	config FlightCheckConfig, minSeparationM float64,
) ([]State, error) {
	if len(members) < 2 || len(members) > 3 || logger == nil || !coordinates.Finite(minSeparationM) || minSeparationM <= 0 {
		return nil, errors.New("invalid flight group")
	}
	if err := config.Validate(); err != nil {
		return nil, err
	}
	for _, member := range members {
		if member.Flight == nil || member.Arming == nil || member.States == nil {
			return nil, errors.New("missing flight group dependency")
		}
	}
	readyCtx, stop := context.WithTimeout(ctx, config.ReadyTimeout)
	origins, err := waitForGroup(readyCtx, members, config.TelemetryMaxAge)
	stop()
	if err != nil {
		return nil, err
	}
	for _, state := range origins {
		if state.Armed || state.Mode != ModeStabilize || !flight.OnGround(state.Position) {
			return nil, errors.New("group must start disarmed on ground in STABILIZE")
		}
	}
	// Консервативная проверка непересекающихся областей для всего маршрута.
	if err := checkSeparation(origins, 2*config.MaxDistanceM+minSeparationM); err != nil {
		return nil, fmt.Errorf("starting positions: %w", err)
	}
	if err := runGroupPhase(ctx, len(members), func(ctx context.Context, i int) error {
		return flight.ObserveStableGround(ctx, members[i].States, origins[i], config.StableDuration, config.TelemetryMaxAge)
	}); err != nil {
		return nil, fmt.Errorf("group stabilization: %w", err)
	}
	return origins, nil
}
