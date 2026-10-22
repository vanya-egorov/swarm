package mavlink

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/streamwriter"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// newArmingTestLink запускает единственного читателя телеметрии и тестовый автопилот.
func newArmingTestLink(t *testing.T) (*ArmingController, *streamwriter.Writer, *frame.Reader) {
	t.Helper()
	source, writer, reader := newServoTestLink(t, streamwriter.V2)
	controller, err := NewArmingController(source, CommandConfig{Timeout: 400 * time.Millisecond, RetryInterval: 80 * time.Millisecond, HeartbeatMaxAge: time.Second, MaxAttempts: 3})
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	sendArmingHeartbeat(t, writer, false)
	if _, err := source.Next(ctx); err != nil {
		t.Fatal(err)
	}
	for range 2 {
		if _, err := reader.Read(); err != nil {
			t.Fatal(err)
		}
	}
	done := make(chan struct{})
	go func() {
		defer close(done)
		for {
			if _, err := source.Next(ctx); err != nil {
				return
			}
		}
	}()
	t.Cleanup(func() { cancel(); <-done })
	return controller, writer, reader
}

// sendArmingHeartbeat передаёт состояние виртуального автопилота в режиме STABILIZE.
func sendArmingHeartbeat(t *testing.T, writer *streamwriter.Writer, armed bool) {
	t.Helper()
	mode := ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	if armed {
		mode |= ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED
	}
	if err := writer.Write(&ardupilotmega.MessageHeartbeat{Type: ardupilotmega.MAV_TYPE_QUADROTOR, Autopilot: ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA, BaseMode: mode}); err != nil {
		t.Fatal(err)
	}
}

// readArmingCommand проверяет адресацию и отсутствие принудительного вооружения.
func readArmingCommand(t *testing.T, reader *frame.Reader, armed bool, attempt uint8) {
	t.Helper()
	packet, err := reader.Read()
	if err != nil {
		t.Fatal(err)
	}
	command, ok := packet.GetMessage().(*ardupilotmega.MessageCommandLong)
	want := float32(0)
	if armed {
		want = 1
	}
	if !ok || command.Command != common.MAV_CMD_COMPONENT_ARM_DISARM || command.Param1 != want || command.Param2 != 0 || command.Confirmation != attempt || command.TargetSystem != 42 || command.TargetComponent != 1 || packet.GetSystemID() != 250 {
		t.Fatalf("unexpected arm command: %+v", packet.GetMessage())
	}
}

// startArmingCommand выполняет команду параллельно с тестовым автопилотом.
func startArmingCommand(controller *ArmingController, armed bool) <-chan error {
	done := make(chan error, 1)
	go func() { done <- controller.SetArmed(context.Background(), armed) }()
	return done
}

func TestArmConfirmation(t *testing.T) {
	t.Parallel()
	controller, writer, reader := newArmingTestLink(t)
	for _, armed := range []bool{true, false} {
		done := startArmingCommand(controller, armed)
		readArmingCommand(t, reader, armed, 0)
		if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: common.MAV_CMD_COMPONENT_ARM_DISARM, Result: ardupilotmega.MAV_RESULT_ACCEPTED, TargetSystem: 250, TargetComponent: 1}); err != nil {
			t.Fatal(err)
		}
		select {
		case err := <-done:
			t.Fatalf("ACK alone completed command: %v", err)
		case <-time.After(30 * time.Millisecond):
		}
		sendArmingHeartbeat(t, writer, armed)
		if err := <-done; err != nil {
			t.Fatal(err)
		}
	}
}

func TestArmRetryAndBusy(t *testing.T) {
	t.Parallel()
	controller, writer, reader := newArmingTestLink(t)
	done := startArmingCommand(controller, true)
	readArmingCommand(t, reader, true, 0)
	if err := controller.SetArmed(context.Background(), false); !errors.Is(err, vehicle.ErrCommandBusy) {
		t.Fatalf("concurrent command: %v", err)
	}
	readArmingCommand(t, reader, true, 1)
	if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: common.MAV_CMD_COMPONENT_ARM_DISARM, Result: ardupilotmega.MAV_RESULT_ACCEPTED}); err != nil {
		t.Fatal(err)
	}
	sendArmingHeartbeat(t, writer, true)
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}

func TestArmACKFiltering(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name                                                 string
		result                                               common.MAV_RESULT
		systemID, componentID, targetSystem, targetComponent uint8
		command                                              common.MAV_CMD
		heartbeat                                            bool
		want                                                 error
	}{
		{"denied", common.MAV_RESULT_DENIED, 42, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, vehicle.ErrCommandRejected},
		{"noState", common.MAV_RESULT_ACCEPTED, 42, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, false, vehicle.ErrCommandTimeout},
		{"progress", common.MAV_RESULT_IN_PROGRESS, 42, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, vehicle.ErrCommandTimeout},
		{"system", common.MAV_RESULT_ACCEPTED, 43, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, vehicle.ErrCommandTimeout},
		{"component", common.MAV_RESULT_ACCEPTED, 42, 2, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, vehicle.ErrCommandTimeout},
		{"targetSystem", common.MAV_RESULT_ACCEPTED, 42, 1, 251, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, vehicle.ErrCommandTimeout},
		{"targetComponent", common.MAV_RESULT_ACCEPTED, 42, 1, 250, 2, common.MAV_CMD_COMPONENT_ARM_DISARM, true, vehicle.ErrCommandTimeout},
		{"command", common.MAV_RESULT_ACCEPTED, 42, 1, 250, 1, common.MAV_CMD_DO_SET_MODE, true, vehicle.ErrCommandTimeout},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			controller, writer, reader := newArmingTestLink(t)
			done := startArmingCommand(controller, true)
			readArmingCommand(t, reader, true, 0)
			writer.SystemID, writer.ComponentID = test.systemID, test.componentID
			if err := writer.Write(&ardupilotmega.MessageCommandAck{Command: test.command, Result: test.result, TargetSystem: test.targetSystem, TargetComponent: test.targetComponent}); err != nil {
				t.Fatal(err)
			}
			writer.SystemID, writer.ComponentID = 42, 1
			if test.heartbeat {
				sendArmingHeartbeat(t, writer, true)
			}
			if err := <-done; !errors.Is(err, test.want) {
				t.Fatalf("SetArmed() = %v, want %v", err, test.want)
			}
		})
	}
}

func TestArmComponentChange(t *testing.T) {
	t.Parallel()
	controller, writer, reader := newArmingTestLink(t)
	done := startArmingCommand(controller, true)
	readArmingCommand(t, reader, true, 0)
	writer.ComponentID = 2
	sendArmingHeartbeat(t, writer, true)
	if err := <-done; !errors.Is(err, vehicle.ErrCommandLinkChanged) {
		t.Fatalf("SetArmed() = %v", err)
	}
}

func TestArmUnavailable(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name  string
		stale bool
		want  error
	}{
		{"cancelled", false, context.Canceled},
		{"stale", true, vehicle.ErrHeartbeatTimeout},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			controller, _, _ := newArmingTestLink(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			if test.stale {
				controller.mu.Lock()
				controller.heartbeat.ReceivedAt = time.Now().Add(-time.Hour)
				controller.mu.Unlock()
			} else {
				cancel()
			}
			if err := controller.SetArmed(ctx, true); !errors.Is(err, test.want) {
				t.Fatalf("SetArmed() = %v, want %v", err, test.want)
			}
		})
	}
}
