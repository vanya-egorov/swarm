package command

import (
	"context"
	"math"
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

func TestVelocityTargetPacket(t *testing.T) {
	controller, _, reader := newArmingTestLink(t)
	controller.mu.Lock()
	channel := controller.channel
	controller.mu.Unlock()
	controller.ObserveHeartbeat(
		channel,
		telemetry.Heartbeat{
			SystemID:    42,
			ComponentID: 1,
			Armed:       true,
			Mode:        telemetry.ModeGuided,
			ReceivedAt:  time.Now(),
		},
	)
	if err := controller.SetVelocityTarget(context.Background(), 0.3, -0.4, 0.1); err != nil {
		t.Fatal(err)
	}
	packet, err := reader.Read()
	if err != nil {
		t.Fatal(err)
	}
	m, ok := packet.GetMessage().(*ardupilotmega.MessageSetPositionTargetLocalNed)
	if !ok || m.TargetSystem != 42 || m.TargetComponent != 1 || m.CoordinateFrame != common.MAV_FRAME_LOCAL_NED ||
		m.TypeMask != 1479 || m.Vx != float32(0.3) || m.Vy != float32(-0.4) || m.Vz != float32(0.1) || m.YawRate != 0 {
		t.Fatalf("wrong navigation velocity packet: %+v", packet.GetMessage())
	}
}

func TestVelocityTargetGuards(t *testing.T) {
	for _, name := range []string{"disarmed", "mode", "stale", "closed", "busy", "cancelled", "nan", "infinite", "speed"} {
		t.Run(name, func(t *testing.T) {
			c := &ArmingController{
				config:    DefaultCommandConfig(),
				heartbeat: telemetry.Heartbeat{Armed: true, Mode: telemetry.ModeGuided, ReceivedAt: time.Now()},
				channel:   &gomavlib.Channel{},
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			n := 0.3
			switch name {
			case "disarmed":
				c.heartbeat.Armed = false
			case "mode":
				c.heartbeat.Mode = telemetry.ModeLand
			case "stale":
				c.heartbeat.ReceivedAt = time.Now().Add(-time.Hour)
			case "closed":
				c.closed = true
			case "busy":
				c.pending = &pendingCommand{}
			case "cancelled":
				cancel()
			case "nan":
				n = math.NaN()
			case "infinite":
				n = math.Inf(1)
			case "speed":
				n = 2
			}
			if err := c.SetVelocityTarget(ctx, n, 0, 0); err == nil {
				t.Fatal("unsafe velocity command accepted")
			}
		})
	}
}

func TestNavigationSpeedIsAcknowledged(t *testing.T) {
	c, writer, reader := newArmingTestLink(t)
	done := make(chan error, 1)
	go func() { done <- c.SetNavigationSpeed(context.Background(), 0.5) }()
	command := readFlightCommand(t, reader, common.MAV_CMD_DO_CHANGE_SPEED)
	if command.Param1 != 1 || command.Param2 != 0.5 || command.Param3 != -1 {
		t.Fatalf("wrong speed command: %+v", command)
	}
	if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: common.MAV_CMD_DO_CHANGE_SPEED, Result: ardupilotmega.MAV_RESULT_ACCEPTED}); err != nil {
		t.Fatal(err)
	}
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}
