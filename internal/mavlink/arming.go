package mavlink

import (
	"context"
	"fmt"
	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/vanya-egorov/swarm/internal/vehicle"
	"sync"
	"time"
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
	return CommandConfig{
		Timeout:         10 * time.Second,
		RetryInterval:   time.Second,
		HeartbeatMaxAge: 5 * time.Second,
		MaxAttempts:     3,
	}
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
	if config.Timeout <= 0 || config.RetryInterval <= 0 || config.HeartbeatMaxAge <= 0 || config.MaxAttempts < 1 ||
		config.MaxAttempts > 255 {
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
