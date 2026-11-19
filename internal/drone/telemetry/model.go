package telemetry

import (
	"context"
	"time"
)

// FlightMode обозначает режим, известный независимому от автопилота ядру.
type FlightMode string

// Режимы, поддерживаемые проверками SITL.
const (
	ModeUnknown   FlightMode = "unknown"
	ModeStabilize FlightMode = "stabilize"
	ModeGuided    FlightMode = "guided"
	ModeLand      FlightMode = "land"
)

// Heartbeat хранит независимые от транспорта данные о доступности дрона.
type Heartbeat struct {
	SystemID    uint8
	ComponentID uint8
	Armed       bool
	Mode        FlightMode
	ReceivedAt  time.Time
}

// isTelemetry отмечает heartbeat как событие телеметрии дрона.
func (Heartbeat) isTelemetry() {}

// Position хранит оценку координат и скорости с явными единицами измерения.
type Position struct {
	SystemID          uint8
	ComponentID       uint8
	LatitudeDeg       float64
	LongitudeDeg      float64
	AltitudeMSLM      float64
	RelativeAltitudeM float64
	NorthVelocityMS   float64
	EastVelocityMS    float64
	DownVelocityMS    float64
	HeadingDeg        float64
	HeadingValid      bool
	BootTime          time.Duration
	ReceivedAt        time.Time
}

// isTelemetry отмечает позицию как событие телеметрии дрона.
func (Position) isTelemetry() {}

// ServoOutputs хранит 16 PWM-значений и порт.
type ServoOutputs struct {
	SystemID    uint8
	ComponentID uint8
	Port        uint8
	PWMUS       [16]uint16
	ReceivedAt  time.Time
}

// isTelemetry отмечает выходы сервоприводов как событие телеметрии дрона.
func (ServoOutputs) isTelemetry() {}

// Telemetry объединяет поддерживаемые события одного дрона без зависимости от MAVLink.
type Telemetry interface {
	isTelemetry()
}

// TelemetryReader задаёт интерфейс чтения событий для сценария наблюдения за дроном.
type TelemetryReader interface {
	Next(context.Context) (Telemetry, error)
}

// StateReader returns a concurrency-safe snapshot of the latest vehicle state.
type StateReader interface {
	Snapshot() State
}

// State хранит последнюю телеметрию дрона.
type State struct {
	SystemID      uint8
	ComponentID   uint8
	Armed         bool
	Mode          FlightMode
	LastHeartbeat time.Time
	Position      *Position
	ServoOutputs  *ServoOutputs
}
