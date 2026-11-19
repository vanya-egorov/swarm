package telemetry

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"sync"
	"time"
)

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
	for ctx.Err() == nil {
		if !time.Now().Before(deadline) {
			return fmt.Errorf("%w: %s", ErrHeartbeatTimeout, m.timeout)
		}
		readCtx, cancel := context.WithDeadline(ctx, deadline)
		telemetry, err := m.reader.Next(readCtx)
		cancel()

		if err != nil {
			if ctx.Err() != nil {
				break
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
	return nil //nolint:nilerr // Отмена контекста — штатная остановка монитора.
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
