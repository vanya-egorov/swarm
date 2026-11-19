package command

import (
	"bufio"
	"context"
	"errors"
	"log/slog"
	"net"
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialect"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/streamwriter"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
	mavtelemetry "github.com/vanya-egorov/swarm/internal/mavlink/telemetry"
)

// Три одновременных канала: команды и подтверждения не должны смешиваться.
func TestThreeVehicleAddressing(t *testing.T) {
	type link struct {
		writer *streamwriter.Writer
		reader *frame.Reader
		done   <-chan error
	}
	links := make([]link, 3)
	for i := range links {
		controller, writer, reader := newArmingSystemLink(t, uint8(i+1))
		// Три канала создаются последовательно; под race это дольше одиночного теста.
		controller.config = CommandConfig{
			Timeout:         3 * time.Second,
			RetryInterval:   100 * time.Millisecond,
			HeartbeatMaxAge: 3 * time.Second,
			MaxAttempts:     20,
		}
		links[i] = link{writer, reader, startArmingCommand(controller, true)}
	}
	for i, l := range links {
		packet, err := l.reader.Read()
		if err != nil {
			t.Fatal(err)
		}
		command, ok := packet.GetMessage().(*ardupilotmega.MessageCommandLong)
		if !ok || command.TargetSystem != uint8(i+1) || command.TargetComponent != 1 ||
			command.Command != common.MAV_CMD_COMPONENT_ARM_DISARM ||
			command.Param1 != 1 {
			t.Fatalf("wrong addressee: %+v", packet.GetMessage())
		}
	}
	ack := &ardupilotmega.MessageCommandAck{
		Command: common.MAV_CMD_COMPONENT_ARM_DISARM,
		Result:  ardupilotmega.MAV_RESULT_ACCEPTED,
	}
	// Чужие ACK и heartbeat на первом порту не завершают команду первого аппарата.
	links[0].writer.SystemID = 2
	if err := links[0].writer.Write(ack); err != nil {
		t.Fatal(err)
	}
	sendArmingHeartbeat(t, links[0].writer, true)
	select {
	case err := <-links[0].done:
		t.Fatalf("foreign system completed command: %v", err)
	case <-time.After(30 * time.Millisecond):
	}
	links[0].writer.SystemID = 1
	for i, l := range links {
		if err := l.writer.Write(ack); err != nil {
			t.Fatal(err)
		}
		sendArmingHeartbeat(t, l.writer, true)
		if err := <-l.done; err != nil {
			t.Fatal(err)
		}
		for _, other := range links[i+1:] {
			select {
			case err := <-other.done:
				t.Fatalf("another vehicle completed: %v", err)
			default:
			}
		}
	}
}

// newArmingTestLink запускает единственного читателя телеметрии и тестовый автопилот.
func newArmingTestLink(t *testing.T) (*ArmingController, *streamwriter.Writer, *frame.Reader) {
	t.Helper()
	return newArmingSystemLink(t, 42)
}

func newArmingSystemLink(t *testing.T, systemID uint8) (*ArmingController, *streamwriter.Writer, *frame.Reader) {
	t.Helper()
	source, writer, reader := newSystemTestLink(t, streamwriter.V2, systemID)
	controller, err := NewArmingController(
		source,
		CommandConfig{
			Timeout:         400 * time.Millisecond,
			RetryInterval:   80 * time.Millisecond,
			HeartbeatMaxAge: time.Second,
			MaxAttempts:     3,
		},
	)
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
	if !ok || command.Command != common.MAV_CMD_COMPONENT_ARM_DISARM || command.Param1 != want || command.Param2 != 0 ||
		command.Confirmation != attempt ||
		command.TargetSystem != 42 ||
		command.TargetComponent != 1 ||
		packet.GetSystemID() != 250 {
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
	if err := controller.SetArmed(context.Background(), false); !errors.Is(err, flight.ErrCommandBusy) {
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
		{"denied", common.MAV_RESULT_DENIED, 42, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, flight.ErrCommandRejected},
		{"noState", common.MAV_RESULT_ACCEPTED, 42, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, false, flight.ErrCommandTimeout},
		{"progress", common.MAV_RESULT_IN_PROGRESS, 42, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, flight.ErrCommandTimeout},
		{"system", common.MAV_RESULT_ACCEPTED, 43, 1, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, flight.ErrCommandTimeout},
		{"component", common.MAV_RESULT_ACCEPTED, 42, 2, 250, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, flight.ErrCommandTimeout},
		{"targetSystem", common.MAV_RESULT_ACCEPTED, 42, 1, 251, 1, common.MAV_CMD_COMPONENT_ARM_DISARM, true, flight.ErrCommandTimeout},
		{"targetComponent", common.MAV_RESULT_ACCEPTED, 42, 1, 250, 2, common.MAV_CMD_COMPONENT_ARM_DISARM, true, flight.ErrCommandTimeout},
		{"command", common.MAV_RESULT_ACCEPTED, 42, 1, 250, 1, common.MAV_CMD_DO_SET_MODE, true, flight.ErrCommandTimeout},
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
	if err := <-done; !errors.Is(err, flight.ErrCommandLinkChanged) {
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
		{"stale", true, telemetry.ErrHeartbeatTimeout},
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
					done <- controller.SetMode(context.Background(), telemetry.ModeGuided)
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
	go func() { done <- controller.SetMode(context.Background(), telemetry.ModeGuided) }()
	readFlightCommand(t, reader, common.MAV_CMD_DO_SET_MODE)
	if err := controller.SetArmed(context.Background(), true); !errors.Is(err, flight.ErrCommandBusy) {
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
	controller.ObserveHeartbeat(
		channel,
		telemetry.Heartbeat{SystemID: 42, ComponentID: 1, Armed: true, Mode: telemetry.ModeGuided, ReceivedAt: time.Now()},
	)
	target := route.PositionTarget{LatitudeDeg: 55.1234567, LongitudeDeg: 37.7654321, RelativeAltitudeM: 1.5}
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
				heartbeat: telemetry.Heartbeat{Armed: true, Mode: telemetry.ModeGuided, ReceivedAt: time.Now()},
				channel:   &gomavlib.Channel{},
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			target := route.PositionTarget{RelativeAltitudeM: 1.5}
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
			case "invalid":
				target.RelativeAltitudeM = 10
			}
			if err := c.SetPositionTarget(ctx, target); err == nil {
				t.Fatal("unsafe target accepted")
			}
		})
	}
}

func newSystemTestLink(
	t *testing.T,
	version streamwriter.Version,
	systemID uint8,
) (*mavtelemetry.TelemetrySource, *streamwriter.Writer, *frame.Reader) {
	t.Helper()
	port, err := net.ListenPacket("udp4", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	address := port.LocalAddr().String()
	if err := port.Close(); err != nil {
		t.Fatal(err)
	}
	source, err := mavtelemetry.NewTelemetrySource(address, systemID, slog.New(slog.DiscardHandler))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(source.Close)
	conn, err := net.Dial("udp4", address)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = conn.Close() })
	if err := conn.SetDeadline(time.Now().Add(3 * time.Second)); err != nil {
		t.Fatal(err)
	}
	dialectRW := &dialect.ReadWriter{Dialect: ardupilotmega.Dialect}
	if err := dialectRW.Initialize(); err != nil {
		t.Fatal(err)
	}
	frameWriter := &frame.Writer{ByteWriter: conn, DialectRW: dialectRW}
	if err := frameWriter.Initialize(); err != nil {
		t.Fatal(err)
	}
	writer := &streamwriter.Writer{FrameWriter: frameWriter, Version: version, SystemID: systemID, ComponentID: 1}
	if err := writer.Initialize(); err != nil {
		t.Fatal(err)
	}
	reader := &frame.Reader{BufByteReader: bufio.NewReader(conn), DialectRW: dialectRW}
	if err := reader.Initialize(); err != nil {
		t.Fatal(err)
	}
	return source, writer, reader
}
