package mavlink

import (
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/streamwriter"
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
