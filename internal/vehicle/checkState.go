package vehicle

import (
	"context"
	"errors"
	"math"
	"time"
)

// observeStableGround ждёт устойчивого наземного состояния перед arm.
func observeStableGround(ctx context.Context, states StateReader, before State, duration, maxAge time.Duration) error {
	if duration == 0 {
		return nil
	}
	timer := time.NewTimer(duration)
	defer timer.Stop()
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		state := states.Snapshot()
		if !freshState(state, maxAge) || !sameVehicle(before, state) || state.Armed || state.Mode != ModeStabilize || !onGround(state.Position) {
			return errors.New("ground state lost")
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-timer.C:
			return nil
		case <-ticker.C:
		}
	}
}

// waitForState ожидает подходящий снимок и учитывает отмену контекста.
func waitForState(ctx context.Context, states StateReader, matches func(State) bool) (State, error) {
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

// freshState проверяет возраст и принадлежность heartbeat, позиции и PWM.
func freshState(state State, maxAge time.Duration) bool {
	if state.Position == nil || state.ServoOutputs == nil || state.ServoOutputs.Port != 0 {
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

// onGround проверяет малую высоту и скорость перед arm.
func onGround(position *Position) bool {
	return grounded(position, 0.5)
}

// grounded проверяет высоту и скорость для выбранного наземного порога.
func grounded(position *Position, maxAltitude float64) bool {
	return position != nil && math.Abs(position.RelativeAltitudeM) < maxAltitude &&
		math.Abs(position.NorthVelocityMS) < 0.2 && math.Abs(position.EastVelocityMS) < 0.2 && math.Abs(position.DownVelocityMS) < 0.2
}

// sameVehicle проверяет системный и компонентный идентификаторы аппарата.
func sameVehicle(before, current State) bool {
	return before.SystemID == current.SystemID && before.ComponentID == current.ComponentID
}
