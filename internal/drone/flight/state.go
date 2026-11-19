package flight

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type State = telemetry.State
type Position = telemetry.Position
type FlightMode = telemetry.FlightMode
type StateReader = telemetry.StateReader

const (
	ModeUnknown   = telemetry.ModeUnknown
	ModeStabilize = telemetry.ModeStabilize
	ModeGuided    = telemetry.ModeGuided
	ModeLand      = telemetry.ModeLand
)

// ArmingController sets arm state and returns after confirmation.
type ArmingController interface {
	SetArmed(context.Context, bool) error
}

// FlightController provides the commands used by bounded flight checks.
type FlightController interface {
	SetMode(context.Context, FlightMode) error
	Takeoff(context.Context, float64) error
	Land(context.Context) error
}

var (
	ErrCommandBusy        = errors.New("command busy")
	ErrCommandRejected    = errors.New("command rejected")
	ErrCommandTimeout     = errors.New("command timeout")
	ErrCommandLinkChanged = errors.New("command link changed")
)

// ObserveStableGround ждёт наземного покоя по реальным часам и часам автопилота.
func ObserveStableGround(ctx context.Context, states StateReader, before State, duration, maxAge time.Duration) error {
	if duration == 0 {
		return nil
	}
	// Замедление симуляции не сокращает проверку IMU. Остановившиеся часы не дают вечного ожидания.
	observeCtx, stop := context.WithTimeout(ctx, 5*duration)
	defer stop()
	started, previousBoot := time.Now(), before.Position.BootTime
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		if err := observeCtx.Err(); err != nil {
			return err
		}
		state := states.Snapshot()
		if !FreshState(state, maxAge) || !SameDrone(before, state) || state.Armed || state.Mode != ModeStabilize ||
			!OnGround(state.Position) {
			return errors.New("ground state lost")
		}
		if state.Position.BootTime < previousBoot {
			return errors.New("autopilot clock moved backwards")
		}
		previousBoot = state.Position.BootTime
		if time.Since(started) >= duration && previousBoot-before.Position.BootTime >= duration {
			return nil
		}
		select {
		case <-observeCtx.Done():
			return observeCtx.Err()
		case <-ticker.C:
		}
	}
}

// WaitForState ожидает подходящий снимок и учитывает отмену контекста.
func WaitForState(ctx context.Context, states StateReader, matches func(State) bool) (State, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		if err := ctx.Err(); err != nil {
			return State{}, err
		}
		state := states.Snapshot()
		if matches(state) {
			return state, nil
		}
		select {
		case <-ctx.Done():
			return State{}, ctx.Err()
		case <-ticker.C:
		}
	}
}

// FreshState проверяет возраст и принадлежность heartbeat, позиции и PWM.
func FreshState(state State, maxAge time.Duration) bool {
	if !coordinates.ValidPosition(state.Position) || state.ServoOutputs == nil || state.ServoOutputs.Port != 0 {
		return false
	}
	if state.SystemID == 0 || state.ComponentID == 0 {
		return false
	}
	if state.Position.SystemID != state.SystemID || state.Position.ComponentID != state.ComponentID {
		return false
	}
	if state.ServoOutputs.SystemID != state.SystemID || state.ServoOutputs.ComponentID != state.ComponentID {
		return false
	}
	for _, timestamp := range []time.Time{state.LastHeartbeat, state.Position.ReceivedAt, state.ServoOutputs.ReceivedAt} {
		age := time.Since(timestamp)
		if timestamp.IsZero() || age < 0 || age > maxAge {
			return false
		}
	}
	return true
}

// OnGround проверяет малую высоту и скорость перед arm.
func OnGround(position *Position) bool {
	return grounded(position, 0.5)
}

// grounded проверяет высоту и скорость для выбранного наземного порога.
func grounded(position *Position, maxAltitude float64) bool {
	return position != nil && math.Abs(position.RelativeAltitudeM) < maxAltitude &&
		math.Abs(
			position.NorthVelocityMS,
		) < 0.2 && math.Abs(position.EastVelocityMS) < 0.2 && math.Abs(position.DownVelocityMS) < 0.2
}

// SameDrone проверяет системный и компонентный идентификаторы аппарата.
func SameDrone(before, current State) bool {
	return before.SystemID == current.SystemID && before.ComponentID == current.ComponentID
}

// RunCommand ограничивает ожидание каждой команды независимым тайм-аутом.
func RunCommand(ctx context.Context, timeout time.Duration, send func(context.Context) error) error {
	commandCtx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()
	if err := commandCtx.Err(); err != nil {
		return context.Cause(commandCtx)
	}
	return send(commandCtx)
}

// CheckState проверяет свежесть, режим и границы полёта одного аппарата.
func CheckState(state, before State, config FlightCheckConfig, armed bool) error {
	if !FreshState(state, config.TelemetryMaxAge) || !SameDrone(before, state) {
		return errors.New("telemetry stale or vehicle changed")
	}
	if state.Armed != armed || state.Mode != ModeGuided {
		return fmt.Errorf("state changed: armed=%t mode=%s (want %t/%s)", state.Armed, state.Mode, armed, ModeGuided)
	}
	if !coordinates.ValidPosition(state.Position) {
		return errors.New("invalid position")
	}
	if state.Position.RelativeAltitudeM < -0.3 || state.Position.RelativeAltitudeM > config.MaxAltitudeM {
		return fmt.Errorf("altitude out of range: %.2f m", state.Position.RelativeAltitudeM)
	}
	if coordinates.HorizontalDistance(
		before.Position.LatitudeDeg,
		before.Position.LongitudeDeg,
		state.Position.LatitudeDeg,
		state.Position.LongitudeDeg,
	) > config.MaxDistanceM {
		return fmt.Errorf("distance exceeds %.1f m", config.MaxDistanceM)
	}
	return nil
}

// WatchFlight наблюдает подъём или удержание и сразу сообщает о выходе за пределы.
func WatchFlight(
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
		if err := CheckState(state, before, config, true); err != nil {
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
