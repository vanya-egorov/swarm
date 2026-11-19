package flight

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"
)

// ArmCheckConfig ограничивает ожидание телеметрии и время работы вооружённого SITL.
type ArmCheckConfig struct {
	ReadyTimeout          time.Duration
	StabilizationDuration time.Duration
	CommandTimeout        time.Duration
	ObservationTimeout    time.Duration
	HeartbeatMaxAge       time.Duration
	HoldDuration          time.Duration
}

// DefaultArmCheckConfig возвращает настройки наземной проверки одного Iris.
func DefaultArmCheckConfig() ArmCheckConfig {
	return ArmCheckConfig{
		ReadyTimeout: 30 * time.Second, StabilizationDuration: 20 * time.Second,
		CommandTimeout: 10 * time.Second, ObservationTimeout: 5 * time.Second,
		HeartbeatMaxAge: 5 * time.Second, HoldDuration: 8 * time.Second,
	}
}

// validate проверяет временные ограничения наземного сценария.
func (c ArmCheckConfig) validate() error {
	if c.ReadyTimeout <= 0 || c.StabilizationDuration < 0 || c.CommandTimeout <= 0 || c.ObservationTimeout <= 0 ||
		c.HeartbeatMaxAge <= 0 ||
		c.HoldDuration < 0 {
		return errors.New("invalid arm timing")
	}
	return nil
}

// RunArmCheck проверяет цикл disarmed → armed → disarmed и изменение PWM без взлёта.
// RunArmCheck всегда пытается разоружить дрон после arm.
func RunArmCheck(
	ctx context.Context,
	controller ArmingController,
	states StateReader,
	logger *slog.Logger,
	config ArmCheckConfig,
) (resultErr error) {
	if controller == nil || states == nil || logger == nil {
		return fmt.Errorf("arm check: missing dependency")
	}
	if err := config.validate(); err != nil {
		return fmt.Errorf("arm check: %w", err)
	}
	readyCtx, cancelReady := context.WithTimeout(ctx, config.ReadyTimeout)
	before, err := WaitForState(
		readyCtx,
		states,
		func(state State) bool { return FreshState(state, config.HeartbeatMaxAge) },
	)
	cancelReady()
	if err != nil {
		return fmt.Errorf("telemetry: %w", err)
	}
	if before.Armed || before.Mode != ModeStabilize || !OnGround(before.Position) {
		return fmt.Errorf(
			"ground state required: armed=%t mode=%s altitude=%.2f m",
			before.Armed,
			before.Mode,
			before.Position.RelativeAltitudeM,
		)
	}
	if err := ObserveStableGround(ctx, states, before, config.StabilizationDuration, config.HeartbeatMaxAge); err != nil {
		return fmt.Errorf("stabilization: %w", err)
	}
	logger.Info("arm baseline", "systemId", before.SystemID, "pwmUs1To4", before.ServoOutputs.PWMUS[:4])
	defer func() {
		cleanupCtx, cancelCleanup := context.WithTimeout(
			context.Background(),
			config.CommandTimeout+config.ObservationTimeout,
		)
		defer cancelCleanup()
		disarmedAt := time.Now()
		if err := controller.SetArmed(cleanupCtx, false); err != nil {
			resultErr = errors.Join(resultErr, fmt.Errorf("disarm: %w", err))
			return
		}
		logger.Info("disarmed", "systemId", before.SystemID)
		if resultErr != nil {
			return
		}
		after, err := WaitForState(cleanupCtx, states, func(state State) bool {
			if !FreshState(state, config.HeartbeatMaxAge) || state.Armed || !SameDrone(before, state) ||
				!state.ServoOutputs.ReceivedAt.After(disarmedAt) {
				return false
			}
			for i := range 4 {
				if state.ServoOutputs.PWMUS[i] != before.ServoOutputs.PWMUS[i] {
					return false
				}
			}
			return true
		})
		if err != nil {
			resultErr = fmt.Errorf("PWM baseline: %w", err)
			return
		}
		logger.Info("arm check complete", "systemId", after.SystemID, "pwmUs1To4", after.ServoOutputs.PWMUS[:4])
	}()
	commandCtx, cancelCommand := context.WithTimeout(ctx, config.CommandTimeout)
	err = controller.SetArmed(commandCtx, true)
	cancelCommand()
	if err != nil {
		return fmt.Errorf("arm: %w", err)
	}
	armedAt := time.Now()
	observeCtx, cancelObserve := context.WithTimeout(ctx, config.ObservationTimeout)
	armed, err := WaitForState(observeCtx, states, func(state State) bool {
		if !FreshState(state, config.HeartbeatMaxAge) || !state.Armed || state.Mode != ModeStabilize ||
			!SameDrone(before, state) ||
			!state.ServoOutputs.ReceivedAt.After(armedAt) {
			return false
		}
		for i := range 4 {
			if state.ServoOutputs.PWMUS[i] <= before.ServoOutputs.PWMUS[i] {
				return false
			}
		}
		return true
	})
	cancelObserve()
	if err != nil {
		return fmt.Errorf("armed PWM: %w", err)
	}
	logger.Info("armed PWM", "systemId", armed.SystemID, "pwmUs1To4", armed.ServoOutputs.PWMUS[:4])
	timer := time.NewTimer(config.HoldDuration)
	defer timer.Stop()
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-timer.C:
			return nil
		case <-ticker.C:
			state := states.Snapshot()
			if !FreshState(state, config.HeartbeatMaxAge) || !SameDrone(before, state) || !state.Armed ||
				state.Mode != ModeStabilize ||
				!OnGround(state.Position) {
				return fmt.Errorf("armed state lost")
			}
		}
	}
}

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

// RunFlightCheckTask runs task after reaching the target altitude and always attempts landing.
func RunFlightCheckTask(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	states StateReader,
	logger *slog.Logger,
	config FlightCheckConfig,
	task func(State) error,
) error {
	return runFlightCheck(ctx, flight, arming, states, logger, config, task)
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
	if err := config.Validate(); err != nil {
		return fmt.Errorf("flight check: %w", err)
	}
	readyCtx, cancelReady := context.WithTimeout(ctx, config.ReadyTimeout)
	before, err := WaitForState(
		readyCtx,
		states,
		func(state State) bool { return FreshState(state, config.TelemetryMaxAge) },
	)
	cancelReady()
	if err != nil {
		return fmt.Errorf("telemetry: %w", err)
	}
	if before.Armed || before.Mode != ModeStabilize || !OnGround(before.Position) {
		return fmt.Errorf(
			"ground state required: armed=%t mode=%s altitude=%.2f m",
			before.Armed,
			before.Mode,
			before.Position.RelativeAltitudeM,
		)
	}
	if err := ObserveStableGround(ctx, states, before, config.StableDuration, config.TelemetryMaxAge); err != nil {
		return fmt.Errorf("stabilization: %w", err)
	}
	if err := RunCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return flight.SetMode(commandCtx, ModeGuided) }); err != nil {
		return fmt.Errorf("GUIDED: %w", err)
	}
	if err := CheckState(states.Snapshot(), before, config, false); err != nil {
		return fmt.Errorf("before arm: %w", err)
	}
	logger.Info("GUIDED ready", "systemId", before.SystemID, "targetAltitudeM", config.TargetAltitudeM)
	defer func() {
		cleanupCtx, cancel := context.WithTimeout(context.Background(), config.LandingTimeout+config.CommandTimeout)
		defer cancel()
		if err := LandAndConfirm(cleanupCtx, flight, arming, states, before, config, logger); err != nil {
			resultErr = errors.Join(resultErr, fmt.Errorf("landing: %w", err))
		}
	}()
	if err := RunCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return arming.SetArmed(commandCtx, true) }); err != nil {
		return fmt.Errorf("arm: %w", err)
	}
	if err := CheckState(states.Snapshot(), before, config, true); err != nil {
		return fmt.Errorf("before takeoff: %w", err)
	}
	if err := RunCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return flight.Takeoff(commandCtx, config.TargetAltitudeM) }); err != nil {
		return fmt.Errorf("takeoff: %w", err)
	}
	climbCtx, cancelClimb := context.WithTimeout(ctx, config.ClimbTimeout)
	reached, err := WatchFlight(
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
	_, err = WatchFlight(holdCtx, states, before, config, func(State) bool { return false })
	cancelHold()
	if ctx.Err() != nil {
		return ctx.Err()
	}
	if err != nil && !errors.Is(err, context.DeadlineExceeded) {
		return fmt.Errorf("hold: %w", err)
	}
	return nil
}

// LandAndConfirm включает LAND при необходимости и ждёт безопасного разоружения на земле.
func LandAndConfirm(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	states StateReader,
	before State,
	config FlightCheckConfig,
	logger *slog.Logger,
) error {
	state := states.Snapshot()
	if FreshState(state, config.TelemetryMaxAge) && SameDrone(before, state) && !state.Armed &&
		landedOnGround(state.Position) {
		return nil
	}
	landErr := RunCommand(ctx, config.CommandTimeout, flight.Land)
	if landErr != nil {
		logger.Warn("LAND unconfirmed", "systemId", before.SystemID, "error", landErr)
	}
	landingCtx, cancel := context.WithTimeout(ctx, config.LandingTimeout)
	defer cancel()
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()
	var groundSince time.Time
	manualDisarmAttempted := false
	var manualDisarmErr error
	for {
		if err := landingCtx.Err(); err != nil {
			return errors.Join(landErr, manualDisarmErr, fmt.Errorf("disarm unconfirmed: %w", err))
		}
		state = states.Snapshot()
		if FreshState(state, config.TelemetryMaxAge) && SameDrone(before, state) {
			if !state.Armed && landedOnGround(state.Position) {
				logger.Info("landed", "systemId", before.SystemID, "altitudeM", state.Position.RelativeAltitudeM)
				return nil
			}
			if state.Armed && landedOnGround(state.Position) {
				if groundSince.IsZero() {
					groundSince = time.Now()
				}
				if !manualDisarmAttempted && time.Since(groundSince) >= 2*time.Second {
					manualDisarmAttempted = true
					manualDisarmErr = RunCommand(
						landingCtx,
						config.CommandTimeout,
						func(commandCtx context.Context) error { return arming.SetArmed(commandCtx, false) },
					)
					if manualDisarmErr != nil {
						logger.Warn("disarm unconfirmed", "systemId", before.SystemID, "error", manualDisarmErr)
					}
				}
			} else {
				groundSince = time.Time{}
			}
		}
		select {
		case <-landingCtx.Done():
		case <-ticker.C:
		}
	}
}

// landedOnGround подтверждает почти нулевую высоту и малые скорости перед disarm.
func landedOnGround(position *Position) bool {
	return grounded(position, 0.15)
}
