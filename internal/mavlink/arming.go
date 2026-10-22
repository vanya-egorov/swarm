package mavlink

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// CommandConfig задаёт ограничения ожидания и повтора команд.
type CommandConfig struct {
	Timeout         time.Duration
	RetryInterval   time.Duration
	HeartbeatMaxAge time.Duration
	MaxAttempts     int
}

// DefaultCommandConfig возвращает настройки коротких команд локального SITL.
func DefaultCommandConfig() CommandConfig {
	return CommandConfig{Timeout: 10 * time.Second, RetryInterval: time.Second, HeartbeatMaxAge: 5 * time.Second, MaxAttempts: 3}
}

// pendingCommand хранит единственную активную команду.
type pendingCommand struct {
	channel     *gomavlib.Channel
	componentID uint8
	command     common.MAV_CMD
	started     time.Time
	ackSeen     bool
	accepted    bool
	err         error
}

// ArmingController сопоставляет COMMAND_ACK и heartbeat одной последовательной операции.
// ArmingController получает события через единственный цикл TelemetrySource.Next.
type ArmingController struct {
	source    *TelemetrySource
	config    CommandConfig
	mu        sync.Mutex
	channel   *gomavlib.Channel
	heartbeat vehicle.Heartbeat
	pending   *pendingCommand
	closed    bool
	changed   chan struct{}
}

// NewArmingController подключает команды до запуска единственного читателя телеметрии.
func NewArmingController(source *TelemetrySource, config CommandConfig) (*ArmingController, error) {
	if source == nil {
		return nil, fmt.Errorf("telemetry source required")
	}
	if config.Timeout <= 0 || config.RetryInterval <= 0 || config.HeartbeatMaxAge <= 0 || config.MaxAttempts < 1 || config.MaxAttempts > 255 {
		return nil, fmt.Errorf("invalid command config: %+v", config)
	}
	if source.controller != nil {
		return nil, fmt.Errorf("controller already attached")
	}
	controller := &ArmingController{source: source, config: config, changed: make(chan struct{}, 1)}
	source.controller = controller
	return controller, nil
}

// SetArmed отправляет обычную arm/disarm без force и ждёт ACK плюс свежий heartbeat.
func (c *ArmingController) SetArmed(ctx context.Context, armed bool) error {
	value := float32(0)
	if armed {
		value = 1
	}
	return c.execute(ctx, common.MAV_CMD_COMPONENT_ARM_DISARM, func(p *pendingCommand, attempt int) error {
		return c.source.node.WriteMessageTo(p.channel, &ardupilotmega.MessageCommandLong{
			TargetSystem: c.source.expectedSystemID, TargetComponent: p.componentID,
			Command: common.MAV_CMD_COMPONENT_ARM_DISARM, Param1: value, Confirmation: uint8(attempt),
		})
	}, func(hb vehicle.Heartbeat) bool { return hb.Armed == armed }, "armed", armed)
}

// execute сопоставляет ACK с командой и при необходимости ждёт нового heartbeat.
func (c *ArmingController) execute(ctx context.Context, command common.MAV_CMD, send func(*pendingCommand, int) error, confirmed func(vehicle.Heartbeat) bool, extra ...any) error {
	ctx, cancel := context.WithTimeout(ctx, c.config.Timeout)
	defer cancel()
	if err := ctx.Err(); err != nil {
		return err
	}
	c.mu.Lock()
	if c.closed {
		c.mu.Unlock()
		return vehicle.ErrCommandLinkChanged
	}
	if c.pending != nil {
		c.mu.Unlock()
		return vehicle.ErrCommandBusy
	}
	if c.channel == nil || c.heartbeat.ReceivedAt.IsZero() || time.Since(c.heartbeat.ReceivedAt) > c.config.HeartbeatMaxAge {
		c.mu.Unlock()
		return fmt.Errorf("%w: before %s", vehicle.ErrHeartbeatTimeout, command)
	}
	pending := &pendingCommand{channel: c.channel, componentID: c.heartbeat.ComponentID, command: command, started: time.Now()}
	c.pending = pending
	c.mu.Unlock()
	defer func() { c.mu.Lock(); c.pending = nil; c.mu.Unlock() }()

	ticker := time.NewTicker(min(c.config.RetryInterval, 100*time.Millisecond))
	defer ticker.Stop()
	attempts := 0
	var lastSend time.Time
	for {
		c.mu.Lock()
		commandErr, accepted, ackSeen, heartbeat := pending.err, pending.accepted, pending.ackSeen, c.heartbeat
		c.mu.Unlock()
		if commandErr != nil {
			return commandErr
		}
		if err := ctx.Err(); err != nil {
			return fmt.Errorf("%w: %s: %w", vehicle.ErrCommandTimeout, command, err)
		}
		if time.Since(heartbeat.ReceivedAt) > c.config.HeartbeatMaxAge {
			return fmt.Errorf("%w: %s", vehicle.ErrHeartbeatTimeout, command)
		}
		if accepted && (confirmed == nil || heartbeat.ReceivedAt.After(pending.started) && confirmed(heartbeat)) {
			attrs := append([]any{"systemId", c.source.expectedSystemID, "componentId", pending.componentID, "command", command.String(), "attempts", attempts}, extra...)
			c.source.logger.Info("command confirmed", attrs...)
			return nil
		}
		if !ackSeen && (lastSend.IsZero() || time.Since(lastSend) >= c.config.RetryInterval) {
			if attempts == c.config.MaxAttempts {
				return fmt.Errorf("%w: %s ACK, %d attempts", vehicle.ErrCommandTimeout, command, attempts)
			}
			if err := send(pending, attempts); err != nil {
				return fmt.Errorf("send %s: %w", command, err)
			}
			attempts++
			lastSend = time.Now()
			attrs := append([]any{"systemId", c.source.expectedSystemID, "componentId", pending.componentID, "command", command.String(), "attempt", attempts}, extra...)
			c.source.logger.Info("command sent", attrs...)
		}
		select {
		case <-ctx.Done():
		case <-c.changed:
		case <-ticker.C:
		}
	}
}

// observeHeartbeat обновляет состояние и отслеживает смену связи.
func (c *ArmingController) observeHeartbeat(channel *gomavlib.Channel, heartbeat vehicle.Heartbeat) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.pending != nil && (c.pending.channel != channel || c.pending.componentID != heartbeat.ComponentID) {
		c.pending.err = vehicle.ErrCommandLinkChanged
	}
	c.channel, c.heartbeat = channel, heartbeat
	c.notify()
}

// observeACK принимает ответ только от адресата команды и для нашего MAVLink-отправителя.
func (c *ArmingController) observeACK(event *gomavlib.EventFrame, ack *ardupilotmega.MessageCommandAck) {
	c.mu.Lock()
	defer c.mu.Unlock()
	pending := c.pending
	if pending == nil || event.Channel != pending.channel || ack.Command != pending.command {
		return
	}
	if event.SystemID() != c.source.expectedSystemID || event.ComponentID() != pending.componentID {
		return
	}
	if (ack.TargetSystem != 0 && ack.TargetSystem != c.source.node.OutSystemID) || (ack.TargetComponent != 0 && ack.TargetComponent != c.source.node.OutComponentID) {
		return
	}
	if pending.accepted || pending.err != nil {
		return
	}
	pending.ackSeen = true
	switch ack.Result {
	case ardupilotmega.MAV_RESULT_ACCEPTED:
		pending.accepted = true
	case ardupilotmega.MAV_RESULT_IN_PROGRESS:
	default:
		pending.err = fmt.Errorf("%w: %s %s (%d)", vehicle.ErrCommandRejected, ack.Command, ack.Result, ack.ResultParam2)
	}
	c.source.logger.Info("command ack", "systemId", event.SystemID(), "componentId", event.ComponentID(), "command", ack.Command.String(), "result", ack.Result.String(), "detail", ack.ResultParam2)
	c.notify()
}

// channelClosed помечает потерю выбранного канала и ожидающей команды.
func (c *ArmingController) channelClosed(channel *gomavlib.Channel) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.channel == channel {
		c.channel = nil
	}
	if c.pending != nil && c.pending.channel == channel {
		c.pending.err = vehicle.ErrCommandLinkChanged
	}
	c.notify()
}

// close запрещает дальнейшие команды и будит ожидающего отправителя.
func (c *ArmingController) close() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.closed = true
	if c.pending != nil {
		c.pending.err = vehicle.ErrCommandLinkChanged
	}
	c.notify()
}

// notify сообщает ожидающему отправителю об изменении состояния без блокировки чтения.
func (c *ArmingController) notify() {
	select {
	case c.changed <- struct{}{}:
	default:
	}
}
