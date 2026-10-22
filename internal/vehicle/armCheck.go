package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"
)

// StateReader предоставляет независимый снимок состояния для сценария проверки.
type StateReader interface{ Snapshot() State }

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
	if c.ReadyTimeout <= 0 || c.StabilizationDuration < 0 || c.CommandTimeout <= 0 || c.ObservationTimeout <= 0 || c.HeartbeatMaxAge <= 0 || c.HoldDuration < 0 {
		return errors.New("invalid arm timing")
	}
	return nil
}

// RunArmCheck проверяет цикл disarmed → armed → disarmed и изменение PWM без взлёта.
// RunArmCheck всегда пытается разоружить дрон после arm.
func RunArmCheck(ctx context.Context, controller ArmingController, states StateReader, logger *slog.Logger, config ArmCheckConfig) (resultErr error) {
	if controller == nil || states == nil || logger == nil {
		return fmt.Errorf("arm check: missing dependency")
	}
	if err := config.validate(); err != nil {
		return fmt.Errorf("arm check: %w", err)
	}
	readyCtx, cancelReady := context.WithTimeout(ctx, config.ReadyTimeout)
	before, err := waitForState(readyCtx, states, func(state State) bool { return freshState(state, config.HeartbeatMaxAge) })
	cancelReady()
	if err != nil {
		return fmt.Errorf("telemetry: %w", err)
	}
	if before.Armed || before.Mode != ModeStabilize || !onGround(before.Position) {
		return fmt.Errorf("ground state required: armed=%t mode=%s altitude=%.2f m", before.Armed, before.Mode, before.Position.RelativeAltitudeM)
	}
	if err := observeStableGround(ctx, states, before, config.StabilizationDuration, config.HeartbeatMaxAge); err != nil {
		return fmt.Errorf("stabilization: %w", err)
	}
	logger.Info("arm baseline", "systemId", before.SystemID, "pwmUs1To4", before.ServoOutputs.PWMUS[:4])
	defer func() {
		cleanupCtx, cancelCleanup := context.WithTimeout(context.Background(), config.CommandTimeout+config.ObservationTimeout)
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
		after, err := waitForState(cleanupCtx, states, func(state State) bool {
			if !freshState(state, config.HeartbeatMaxAge) || state.Armed || !sameVehicle(before, state) || !state.ServoOutputs.ReceivedAt.After(disarmedAt) {
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
	armed, err := waitForState(observeCtx, states, func(state State) bool {
		if !freshState(state, config.HeartbeatMaxAge) || !state.Armed || state.Mode != ModeStabilize || !sameVehicle(before, state) || !state.ServoOutputs.ReceivedAt.After(armedAt) {
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
			if !freshState(state, config.HeartbeatMaxAge) || !sameVehicle(before, state) || !state.Armed || state.Mode != ModeStabilize || !onGround(state.Position) {
				return fmt.Errorf("armed state lost")
			}
		}
	}
}
