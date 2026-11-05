package vehicle

import (
	"context"
	"errors"
)

// ArmingController изменяет armed и возвращает успех только после подтверждения состояния.
type ArmingController interface {
	SetArmed(context.Context, bool) error
}

// FlightController задаёт минимальные команды для ограниченного полёта.
type FlightController interface {
	SetMode(context.Context, FlightMode) error
	Takeoff(context.Context, float64) error
	Land(context.Context) error
}

// Ошибки выполнения команд автопилота.
var (
	ErrCommandBusy        = errors.New("command busy")
	ErrCommandRejected    = errors.New("command rejected")
	ErrCommandTimeout     = errors.New("command timeout")
	ErrCommandLinkChanged = errors.New("command link changed")
)
