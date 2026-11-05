package mavlink

import (
	"fmt"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/vanya-egorov/swarm/internal/vehicle"
)

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
	if (ack.TargetSystem != 0 && ack.TargetSystem != c.source.node.OutSystemID) ||
		(ack.TargetComponent != 0 && ack.TargetComponent != c.source.node.OutComponentID) {
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
		pending.err = fmt.Errorf(
			"%w: %s %s (%d)",
			vehicle.ErrCommandRejected,
			ack.Command,
			ack.Result,
			ack.ResultParam2,
		)
	}
	c.source.logger.Info(
		"command ack",
		"systemId",
		event.SystemID(),
		"componentId",
		event.ComponentID(),
		"command",
		ack.Command.String(),
		"result",
		ack.Result.String(),
		"detail",
		ack.ResultParam2,
	)
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
