package mavlink

import (
	"context"
	"fmt"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// execute сопоставляет ACK с командой и при необходимости ждёт нового heartbeat.
func (c *ArmingController) execute(
	ctx context.Context,
	command common.MAV_CMD,
	send func(*pendingCommand, int) error,
	confirmed func(vehicle.Heartbeat) bool,
	extra ...any,
) error {
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
	if c.channel == nil || c.heartbeat.ReceivedAt.IsZero() ||
		time.Since(c.heartbeat.ReceivedAt) > c.config.HeartbeatMaxAge {
		c.mu.Unlock()
		return fmt.Errorf("%w: before %s", vehicle.ErrHeartbeatTimeout, command)
	}
	pending := &pendingCommand{
		channel:     c.channel,
		componentID: c.heartbeat.ComponentID,
		command:     command,
		started:     time.Now(),
	}
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
			attrs := append(
				[]any{
					"systemId",
					c.source.expectedSystemID,
					"componentId",
					pending.componentID,
					"command",
					command.String(),
					"attempts",
					attempts,
				},
				extra...)
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
			attrs := append(
				[]any{
					"systemId",
					c.source.expectedSystemID,
					"componentId",
					pending.componentID,
					"command",
					command.String(),
					"attempt",
					attempts,
				},
				extra...)
			c.source.logger.Info("command sent", attrs...)
		}
		select {
		case <-ctx.Done():
		case <-c.changed:
		case <-ticker.C:
		}
	}
}
