package mavlink

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/streamwriter"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// sendFlightHeartbeat передаёт режим и armed тестового ArduCopter.
func sendFlightHeartbeat(t *testing.T, writer *streamwriter.Writer, mode ardupilotmega.COPTER_MODE, armed bool) {
	t.Helper()
	base := ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	if armed {
		base |= ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED
	}
	if err := writer.Write(&ardupilotmega.MessageHeartbeat{Type: ardupilotmega.MAV_TYPE_QUADROTOR, Autopilot: ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA, BaseMode: base, CustomMode: uint32(mode)}); err != nil {
		t.Fatal(err)
	}
}

// readFlightCommand получает адресованный COMMAND_INT от адаптера.
func readFlightCommand(t *testing.T, reader *frame.Reader, want common.MAV_CMD) *ardupilotmega.MessageCommandInt {
	t.Helper()
	packet, err := reader.Read()
	if err != nil {
		t.Fatal(err)
	}
	command, ok := packet.GetMessage().(*ardupilotmega.MessageCommandInt)
	if !ok || command.Command != want || command.TargetSystem != 42 || command.TargetComponent != 1 ||
		packet.GetSystemID() != 250 {
		t.Fatalf("unexpected flight command: %+v", packet.GetMessage())
	}
	return command
}

func TestFlightModeConfirmation(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name    string
		command common.MAV_CMD
		mode    ardupilotmega.COPTER_MODE
	}{
		{"guided", common.MAV_CMD_DO_SET_MODE, ardupilotmega.COPTER_MODE_GUIDED},
		{"land", common.MAV_CMD_NAV_LAND, ardupilotmega.COPTER_MODE_LAND},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			controller, writer, reader := newArmingTestLink(t)
			done := make(chan error, 1)
			go func() {
				if test.mode == ardupilotmega.COPTER_MODE_GUIDED {
					done <- controller.SetMode(context.Background(), vehicle.ModeGuided)
				} else {
					done <- controller.Land(context.Background())
				}
			}()
			command := readFlightCommand(t, reader, test.command)
			if test.command == common.MAV_CMD_DO_SET_MODE &&
				(command.Param1 != float32(common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) || command.Param2 != float32(ardupilotmega.COPTER_MODE_GUIDED)) {
				t.Fatalf("wrong mode parameters: %+v", command)
			}
			if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: test.command, Result: ardupilotmega.MAV_RESULT_ACCEPTED, TargetSystem: 250, TargetComponent: 1}); err != nil {
				t.Fatal(err)
			}
			select {
			case err := <-done:
				t.Fatalf("ACK without mode completed command: %v", err)
			case <-time.After(30 * time.Millisecond):
			}
			sendFlightHeartbeat(t, writer, test.mode, false)
			if err := <-done; err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestTakeoffCommand(t *testing.T) {
	t.Parallel()
	controller, writer, reader := newArmingTestLink(t)
	done := make(chan error, 1)
	go func() { done <- controller.Takeoff(context.Background(), 1.5) }()
	command := readFlightCommand(t, reader, common.MAV_CMD_NAV_TAKEOFF)
	if command.Frame != common.MAV_FRAME_GLOBAL_RELATIVE_ALT || command.Z != 1.5 {
		t.Fatalf("wrong takeoff altitude/frame: %+v", command)
	}
	select {
	case err := <-done:
		t.Fatalf("takeoff completed without ACK: %v", err)
	case <-time.After(30 * time.Millisecond):
	}
	if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: common.MAV_CMD_NAV_TAKEOFF, Result: ardupilotmega.MAV_RESULT_ACCEPTED}); err != nil {
		t.Fatal(err)
	}
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}

func TestFlightCommandBusy(t *testing.T) {
	t.Parallel()
	controller, writer, reader := newArmingTestLink(t)
	done := make(chan error, 1)
	go func() { done <- controller.SetMode(context.Background(), vehicle.ModeGuided) }()
	readFlightCommand(t, reader, common.MAV_CMD_DO_SET_MODE)
	if err := controller.SetArmed(context.Background(), true); !errors.Is(err, vehicle.ErrCommandBusy) {
		t.Fatalf("SetArmed() = %v, want busy", err)
	}
	if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: common.MAV_CMD_DO_SET_MODE, Result: ardupilotmega.MAV_RESULT_ACCEPTED}); err != nil {
		t.Fatal(err)
	}
	sendFlightHeartbeat(t, writer, ardupilotmega.COPTER_MODE_GUIDED, false)
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}

func TestPositionTargetPacket(t *testing.T) {
	t.Parallel()
	controller, _, reader := newArmingTestLink(t)
	controller.mu.Lock()
	channel := controller.channel
	controller.mu.Unlock()
	controller.observeHeartbeat(
		channel,
		vehicle.Heartbeat{SystemID: 42, ComponentID: 1, Armed: true, Mode: vehicle.ModeGuided, ReceivedAt: time.Now()},
	)
	target := vehicle.PositionTarget{LatitudeDeg: 55.1234567, LongitudeDeg: 37.7654321, RelativeAltitudeM: 1.5}
	if err := controller.SetPositionTarget(context.Background(), target); err != nil {
		t.Fatal(err)
	}
	packet, err := reader.Read()
	if err != nil {
		t.Fatal(err)
	}
	msg, ok := packet.GetMessage().(*ardupilotmega.MessageSetPositionTargetGlobalInt)
	if !ok {
		t.Fatalf("unexpected message: %T", packet.GetMessage())
	}
	if msg.TargetSystem != 42 || msg.TargetComponent != 1 || packet.GetSystemID() != 250 ||
		msg.CoordinateFrame != common.MAV_FRAME_GLOBAL_RELATIVE_ALT ||
		msg.TypeMask != 3576 ||
		msg.LatInt != 551234567 ||
		msg.LonInt != 377654321 ||
		msg.Alt != 1.5 {
		t.Fatalf("wrong position target: %+v", msg)
	}
}

func TestPositionTargetGuards(t *testing.T) {
	for _, name := range []string{"disarmed", "mode", "stale", "closed", "busy", "cancelled", "invalid"} {
		t.Run(name, func(t *testing.T) {
			c := &ArmingController{
				config:    DefaultCommandConfig(),
				heartbeat: vehicle.Heartbeat{Armed: true, Mode: vehicle.ModeGuided, ReceivedAt: time.Now()},
				channel:   &gomavlib.Channel{},
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			target := vehicle.PositionTarget{RelativeAltitudeM: 1.5}
			switch name {
			case "disarmed":
				c.heartbeat.Armed = false
			case "mode":
				c.heartbeat.Mode = vehicle.ModeLand
			case "stale":
				c.heartbeat.ReceivedAt = time.Now().Add(-time.Hour)
			case "closed":
				c.closed = true
			case "busy":
				c.pending = &pendingCommand{}
			case "cancelled":
				cancel()
			case "invalid":
				target.RelativeAltitudeM = 10
			}
			if err := c.SetPositionTarget(ctx, target); err == nil {
				t.Fatal("unsafe target accepted")
			}
		})
	}
}
