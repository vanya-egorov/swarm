package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
)

// RunFlightCheck проверяет взлёт и посадку одного локального SITL по свежей телеметрии.
func RunFlightCheck(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	states StateReader,
	logger *slog.Logger,
	config FlightCheckConfig,
) error {
	return runFlightCheck(ctx, flight, arming, states, logger, config, nil)
}

// runFlightCheck выполняет дополнительную задачу после подъёма с общей процедурой посадки.
func runFlightCheck(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	states StateReader,
	logger *slog.Logger,
	config FlightCheckConfig,
	task func(State) error,
) (resultErr error) {
	if flight == nil || arming == nil || states == nil || logger == nil {
		return fmt.Errorf("flight check: missing dependency")
	}
	if err := config.validate(); err != nil {
		return fmt.Errorf("flight check: %w", err)
	}
	readyCtx, cancelReady := context.WithTimeout(ctx, config.ReadyTimeout)
	before, err := waitForState(
		readyCtx,
		states,
		func(state State) bool { return freshState(state, config.TelemetryMaxAge) },
	)
	cancelReady()
	if err != nil {
		return fmt.Errorf("telemetry: %w", err)
	}
	if before.Armed || before.Mode != ModeStabilize || !onGround(before.Position) {
		return fmt.Errorf(
			"ground state required: armed=%t mode=%s altitude=%.2f m",
			before.Armed,
			before.Mode,
			before.Position.RelativeAltitudeM,
		)
	}
	if err := observeStableGround(ctx, states, before, config.StableDuration, config.TelemetryMaxAge); err != nil {
		return fmt.Errorf("stabilization: %w", err)
	}
	if err := flightCheckCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return flight.SetMode(commandCtx, ModeGuided) }); err != nil {
		return fmt.Errorf("GUIDED: %w", err)
	}
	if err := flightCheckState(states.Snapshot(), before, config, false); err != nil {
		return fmt.Errorf("before arm: %w", err)
	}
	logger.Info("GUIDED ready", "systemId", before.SystemID, "targetAltitudeM", config.TargetAltitudeM)
	defer func() {
		cleanupCtx, cancel := context.WithTimeout(context.Background(), config.LandingTimeout+config.CommandTimeout)
		defer cancel()
		if err := landAndConfirm(cleanupCtx, flight, arming, states, before, config, logger); err != nil {
			resultErr = errors.Join(resultErr, fmt.Errorf("landing: %w", err))
		}
	}()
	if err := flightCheckCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return arming.SetArmed(commandCtx, true) }); err != nil {
		return fmt.Errorf("arm: %w", err)
	}
	if err := flightCheckState(states.Snapshot(), before, config, true); err != nil {
		return fmt.Errorf("before takeoff: %w", err)
	}
	if err := flightCheckCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return flight.Takeoff(commandCtx, config.TargetAltitudeM) }); err != nil {
		return fmt.Errorf("takeoff: %w", err)
	}
	climbCtx, cancelClimb := context.WithTimeout(ctx, config.ClimbTimeout)
	reached, err := watchFlight(
		climbCtx,
		states,
		before,
		config,
		func(state State) bool { return state.Position.RelativeAltitudeM >= config.TargetAltitudeM-0.3 },
	)
	cancelClimb()
	if err != nil {
		return fmt.Errorf("climb: %w", err)
	}
	logger.Info("altitude reached", "systemId", before.SystemID, "altitudeM", reached.Position.RelativeAltitudeM)
	if task != nil {
		if err := task(before); err != nil {
			return fmt.Errorf("route: %w", err)
		}
	}

	holdCtx, cancelHold := context.WithTimeout(ctx, config.HoldDuration)
	_, err = watchFlight(holdCtx, states, before, config, func(State) bool { return false })
	cancelHold()
	if ctx.Err() != nil {
		return ctx.Err()
	}
	if err != nil && !errors.Is(err, context.DeadlineExceeded) {
		return fmt.Errorf("hold: %w", err)
	}
	return nil
}
