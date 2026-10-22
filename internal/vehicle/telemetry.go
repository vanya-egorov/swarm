package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"sync"
	"time"
)

// FlightMode обозначает режим, известный независимому от автопилота ядру.
type FlightMode string

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

// ErrHeartbeatTimeout обозначает потерю heartbeat выбранного дрона.
var ErrHeartbeatTimeout = errors.New("heartbeat timeout")

// ErrSystemIDChanged обозначает получение телеметрии другого дрона тем же монитором.
var ErrSystemIDChanged = errors.New("system ID changed")

// Monitor наблюдает за одним дроном и не отправляет полётные команды.
type Monitor struct {
	reader  TelemetryReader
	logger  *slog.Logger
	timeout time.Duration
	mu      sync.RWMutex
	state   State
}

// NewMonitor создаёт монитор с заданным источником телеметрии и тайм-аутом heartbeat.
func NewMonitor(reader TelemetryReader, logger *slog.Logger, timeout time.Duration) (*Monitor, error) {
	if reader == nil {
		return nil, errors.New("telemetry reader required")
	}
	if logger == nil {
		return nil, errors.New("logger required")
	}
	if timeout <= 0 {
		return nil, fmt.Errorf("invalid heartbeat timeout: %s", timeout)
	}

	return &Monitor{reader: reader, logger: logger, timeout: timeout}, nil
}

// Snapshot возвращает копию последнего известного состояния дрона.
func (m *Monitor) Snapshot() State {
	m.mu.RLock()
	defer m.mu.RUnlock()
	state := m.state
	if state.Position != nil {
		position := *state.Position
		state.Position = &position
	}
	if state.ServoOutputs != nil {
		outputs := *state.ServoOutputs
		state.ServoOutputs = &outputs
	}
	return state
}

// Run читает телеметрию до отмены контекста, ошибки источника или потери heartbeat.
func (m *Monitor) Run(ctx context.Context) error {
	deadline := time.Now().Add(m.timeout)
	for {
		if ctx.Err() != nil {
			return nil
		}
		if !time.Now().Before(deadline) {
			return fmt.Errorf("%w: %s", ErrHeartbeatTimeout, m.timeout)
		}
		readCtx, cancel := context.WithDeadline(ctx, deadline)
		telemetry, err := m.reader.Next(readCtx)
		cancel()

		if err != nil {
			if ctx.Err() != nil {
				return nil
			}
			if errors.Is(err, context.DeadlineExceeded) {
				return fmt.Errorf("%w: %s", ErrHeartbeatTimeout, m.timeout)
			}
			return fmt.Errorf("telemetry: %w", err)
		}

		switch event := telemetry.(type) {
		case Heartbeat:
			m.mu.Lock()
			if err := ensureSystemID(m.state.SystemID, event.SystemID); err != nil {
				m.mu.Unlock()
				return err
			}
			if m.state.ComponentID != 0 && m.state.ComponentID != event.ComponentID {
				m.state.Position = nil
				m.state.ServoOutputs = nil
			}
			m.state.SystemID = event.SystemID
			m.state.ComponentID = event.ComponentID
			m.state.Armed = event.Armed
			m.state.Mode = event.Mode
			m.state.LastHeartbeat = event.ReceivedAt
			m.mu.Unlock()
			deadline = time.Now().Add(m.timeout)
			m.logger.Info("heartbeat",
				"systemId", event.SystemID,
				"componentId", event.ComponentID,
				"armed", event.Armed,
				"mode", event.Mode,
				"receivedAt", event.ReceivedAt,
			)
		case Position:
			m.mu.Lock()
			if err := ensureSystemID(m.state.SystemID, event.SystemID); err != nil {
				m.mu.Unlock()
				return err
			}
			m.state.SystemID = event.SystemID
			m.state.ComponentID = event.ComponentID
			position := event
			m.state.Position = &position
			m.mu.Unlock()
			m.logger.Info("position",
				"systemId", event.SystemID,
				"componentId", event.ComponentID,
				"latitudeDeg", event.LatitudeDeg,
				"longitudeDeg", event.LongitudeDeg,
				"altitudeMslM", event.AltitudeMSLM,
				"relativeAltitudeM", event.RelativeAltitudeM,
				"northVelocityMps", event.NorthVelocityMS,
				"eastVelocityMps", event.EastVelocityMS,
				"downVelocityMps", event.DownVelocityMS,
				"headingDeg", event.HeadingDeg,
				"headingValid", event.HeadingValid,
				"bootTimeMs", event.BootTime.Milliseconds(),
				"receivedAt", event.ReceivedAt,
			)
		case ServoOutputs:
			m.mu.Lock()
			if err := ensureSystemID(m.state.SystemID, event.SystemID); err != nil {
				m.mu.Unlock()
				return err
			}
			m.state.SystemID = event.SystemID
			m.state.ComponentID = event.ComponentID
			outputs := event
			m.state.ServoOutputs = &outputs
			m.mu.Unlock()
			m.logger.Info("servo outputs",
				"systemId", event.SystemID,
				"componentId", event.ComponentID,
				"port", event.Port,
				"pwmUs1To4", event.PWMUS[:4],
				"receivedAt", event.ReceivedAt,
			)
		default:
			return fmt.Errorf("unsupported telemetry: %T", telemetry)
		}
	}
}

// ensureSystemID проверяет, что монитор продолжает получать данные одного дрона.
func ensureSystemID(current, incoming uint8) error {
	if incoming == 0 {
		return fmt.Errorf("%w: incoming ID is zero", ErrSystemIDChanged)
	}
	if current != 0 && current != incoming {
		return fmt.Errorf("%w: current %d, incoming %d", ErrSystemIDChanged, current, incoming)
	}
	return nil
}
