package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"time"
)

// FlightCheckConfig ограничивает высоту, ожидания и длительность пробного полёта SITL.
type FlightCheckConfig struct {
	ReadyTimeout    time.Duration
	StableDuration  time.Duration
	CommandTimeout  time.Duration
	ClimbTimeout    time.Duration
	HoldDuration    time.Duration
	LandingTimeout  time.Duration
	TelemetryMaxAge time.Duration
	TargetAltitudeM float64
	MaxAltitudeM    float64
	MaxDistanceM    float64
}

// DefaultFlightCheckConfig задаёт короткий полёт одного Iris на 1,5 метра.
func DefaultFlightCheckConfig() FlightCheckConfig {
	return FlightCheckConfig{
		ReadyTimeout: 30 * time.Second, StableDuration: 20 * time.Second,
		CommandTimeout: 10 * time.Second, ClimbTimeout: 25 * time.Second,
		HoldDuration: 2 * time.Second, LandingTimeout: 60 * time.Second,
		TelemetryMaxAge: 5 * time.Second, TargetAltitudeM: 1.5,
		MaxAltitudeM: 2.5, MaxDistanceM: 5,
	}
}

// validate проверяет время и пространственные границы полёта.
func (c FlightCheckConfig) validate() error {
	if c.ReadyTimeout <= 0 || c.StableDuration < 0 || c.CommandTimeout <= 0 || c.ClimbTimeout <= 0 ||
		c.HoldDuration < 0 || c.LandingTimeout <= 0 || c.TelemetryMaxAge <= 0 {
		return errors.New("invalid flight timing")
	}
	if !finite(c.TargetAltitudeM) || c.TargetAltitudeM < 0.8 || c.TargetAltitudeM > 2 {
		return fmt.Errorf("invalid target altitude: %g m", c.TargetAltitudeM)
	}
	if !finite(c.MaxAltitudeM) || c.MaxAltitudeM <= c.TargetAltitudeM {
		return fmt.Errorf("invalid altitude limit: %g m", c.MaxAltitudeM)
	}
	if !finite(c.MaxDistanceM) || c.MaxDistanceM <= 0 {
		return fmt.Errorf("invalid distance limit: %g m", c.MaxDistanceM)
	}
	return nil
}

// finite проверяет, что измеримый предел не равен NaN или бесконечности.
func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

// RunFlightCheck проверяет взлёт и посадку одного локального SITL по свежей телеметрии.
func RunFlightCheck(ctx context.Context, flight FlightController, arming ArmingController, states StateReader, logger *slog.Logger, config FlightCheckConfig) (resultErr error) {
	if flight == nil || arming == nil || states == nil || logger == nil {
		return fmt.Errorf("flight check: missing dependency")
	}
	if err := config.validate(); err != nil {
		return fmt.Errorf("flight check: %w", err)
	}
	readyCtx, cancelReady := context.WithTimeout(ctx, config.ReadyTimeout)
	before, err := waitForState(readyCtx, states, func(state State) bool { return freshState(state, config.TelemetryMaxAge) })
	cancelReady()
	if err != nil {
		return fmt.Errorf("telemetry: %w", err)
	}
	if before.Armed || before.Mode != ModeStabilize || !onGround(before.Position) {
		return fmt.Errorf("ground state required: armed=%t mode=%s altitude=%.2f m", before.Armed, before.Mode, before.Position.RelativeAltitudeM)
	}
	if err := observeStableGround(ctx, states, before, config.StableDuration, config.TelemetryMaxAge); err != nil {
		return fmt.Errorf("stabilization: %w", err)
	}
	if err := flightCheckCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return flight.SetMode(commandCtx, ModeGuided) }); err != nil {
		return fmt.Errorf("GUIDED: %w", err)
	}
	if err := flightCheckState(states.Snapshot(), before, config, false, ModeGuided); err != nil {
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
	if err := flightCheckState(states.Snapshot(), before, config, true, ModeGuided); err != nil {
		return fmt.Errorf("before takeoff: %w", err)
	}
	if err := flightCheckCommand(ctx, config.CommandTimeout, func(commandCtx context.Context) error { return flight.Takeoff(commandCtx, config.TargetAltitudeM) }); err != nil {
		return fmt.Errorf("takeoff: %w", err)
	}
	climbCtx, cancelClimb := context.WithTimeout(ctx, config.ClimbTimeout)
	reached, err := watchFlight(climbCtx, states, before, config, func(state State) bool { return state.Position.RelativeAltitudeM >= config.TargetAltitudeM-0.3 })
	cancelClimb()
	if err != nil {
		return fmt.Errorf("climb: %w", err)
	}
	logger.Info("altitude reached", "systemId", before.SystemID, "altitudeM", reached.Position.RelativeAltitudeM)
	holdCtx, cancelHold := context.WithTimeout(ctx, config.HoldDuration)
	_, err = watchFlight(holdCtx, states, before, config, func(State) bool { return false })
	cancelHold()
	if err != nil && !errors.Is(err, context.DeadlineExceeded) {
		return fmt.Errorf("hold: %w", err)
	}
	return nil
}

// flightCheckCommand ограничивает ожидание каждой команды независимым тайм-аутом.
func flightCheckCommand(ctx context.Context, timeout time.Duration, send func(context.Context) error) error {
	commandCtx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()
	return send(commandCtx)
}

// flightCheckState проверяет свежесть, режим и границы полёта одного аппарата.
func flightCheckState(state, before State, config FlightCheckConfig, armed bool, mode FlightMode) error {
	if !freshState(state, config.TelemetryMaxAge) || !sameVehicle(before, state) {
		return errors.New("telemetry stale or vehicle changed")
	}
	if state.Armed != armed || state.Mode != mode {
		return fmt.Errorf("state changed: armed=%t mode=%s (want %t/%s)", state.Armed, state.Mode, armed, mode)
	}
	if state.Position.RelativeAltitudeM < -0.3 || state.Position.RelativeAltitudeM > config.MaxAltitudeM {
		return fmt.Errorf("altitude out of range: %.2f m", state.Position.RelativeAltitudeM)
	}
	lat := (state.Position.LatitudeDeg - before.Position.LatitudeDeg) * 111195
	lon := (state.Position.LongitudeDeg - before.Position.LongitudeDeg) * 111195 * math.Cos(before.Position.LatitudeDeg*math.Pi/180)
	if math.Hypot(lat, lon) > config.MaxDistanceM {
		return fmt.Errorf("distance exceeds %.1f m", config.MaxDistanceM)
	}
	return nil
}

// watchFlight наблюдает подъём или удержание и сразу сообщает о выходе за пределы.
func watchFlight(ctx context.Context, states StateReader, before State, config FlightCheckConfig, done func(State) bool) (State, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		if err := ctx.Err(); err != nil {
			return State{}, err
		}
		state := states.Snapshot()
		if err := flightCheckState(state, before, config, true, ModeGuided); err != nil {
			return State{}, err
		}
		if done(state) {
			return state, nil
		}
		select {
		case <-ctx.Done():
			return State{}, ctx.Err()
		case <-ticker.C:
		}
	}
}

// landAndConfirm включает LAND при необходимости и ждёт безопасного разоружения на земле.
func landAndConfirm(ctx context.Context, flight FlightController, arming ArmingController, states StateReader, before State, config FlightCheckConfig, logger *slog.Logger) error {
	state := states.Snapshot()
	if freshState(state, config.TelemetryMaxAge) && sameVehicle(before, state) && !state.Armed && landedOnGround(state.Position) {
		return nil
	}
	landErr := flightCheckCommand(ctx, config.CommandTimeout, flight.Land)
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
		if freshState(state, config.TelemetryMaxAge) && sameVehicle(before, state) {
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
					manualDisarmErr = flightCheckCommand(landingCtx, config.CommandTimeout, func(commandCtx context.Context) error { return arming.SetArmed(commandCtx, false) })
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
