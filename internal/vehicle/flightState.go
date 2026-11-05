package vehicle

import (
	"context"
	"errors"
	"fmt"
	"time"
)

// flightCheckCommand ограничивает ожидание каждой команды независимым тайм-аутом.
func flightCheckCommand(ctx context.Context, timeout time.Duration, send func(context.Context) error) error {
	commandCtx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()
	if err := commandCtx.Err(); err != nil {
		return context.Cause(commandCtx)
	}
	return send(commandCtx)
}

// flightCheckState проверяет свежесть, режим и границы полёта одного аппарата.
func flightCheckState(state, before State, config FlightCheckConfig, armed bool) error {
	if !freshState(state, config.TelemetryMaxAge) || !sameVehicle(before, state) {
		return errors.New("telemetry stale or vehicle changed")
	}
	if state.Armed != armed || state.Mode != ModeGuided {
		return fmt.Errorf("state changed: armed=%t mode=%s (want %t/%s)", state.Armed, state.Mode, armed, ModeGuided)
	}
	if !validPosition(state.Position) {
		return errors.New("invalid position")
	}
	if state.Position.RelativeAltitudeM < -0.3 || state.Position.RelativeAltitudeM > config.MaxAltitudeM {
		return fmt.Errorf("altitude out of range: %.2f m", state.Position.RelativeAltitudeM)
	}
	if horizontalDistance(
		before.Position.LatitudeDeg,
		before.Position.LongitudeDeg,
		state.Position.LatitudeDeg,
		state.Position.LongitudeDeg,
	) > config.MaxDistanceM {
		return fmt.Errorf("distance exceeds %.1f m", config.MaxDistanceM)
	}
	return nil
}

// watchFlight наблюдает подъём или удержание и сразу сообщает о выходе за пределы.
func watchFlight(
	ctx context.Context,
	states StateReader,
	before State,
	config FlightCheckConfig,
	done func(State) bool,
) (State, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		if err := ctx.Err(); err != nil {
			return State{}, err
		}
		state := states.Snapshot()
		if err := flightCheckState(state, before, config, true); err != nil {
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
