package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
)

// FlightMember объединяет управление и телеметрию одного участника.
type FlightMember struct {
	Flight FlightController
	Arming ArmingController
	States StateReader
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
		_, err := watchFlight(holdCtx, members[i].States, origins[i], config, func(State) bool { return false })
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
	if len(members) < 2 || len(members) > 3 || logger == nil || !finite(minSeparationM) || minSeparationM <= 0 {
		return nil, errors.New("invalid flight group")
	}
	if err := config.validate(); err != nil {
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
		if state.Armed || state.Mode != ModeStabilize || !onGround(state.Position) {
			return nil, errors.New("group must start disarmed on ground in STABILIZE")
		}
	}
	// Консервативная проверка непересекающихся областей для всего маршрута.
	if err := checkSeparation(origins, 2*config.MaxDistanceM+minSeparationM); err != nil {
		return nil, fmt.Errorf("starting positions: %w", err)
	}
	if err := runGroupPhase(ctx, len(members), func(ctx context.Context, i int) error {
		return observeStableGround(ctx, members[i].States, origins[i], config.StableDuration, config.TelemetryMaxAge)
	}); err != nil {
		return nil, fmt.Errorf("group stabilization: %w", err)
	}
	return origins, nil
}
