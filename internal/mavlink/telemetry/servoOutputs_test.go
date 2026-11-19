package telemetry

import (
	"bufio"
	"context"
	"log/slog"
	"net"
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialect"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/bluenviron/gomavlib/v3/pkg/streamwriter"

	model "github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

// newServoTestLink создаёт локальный UDP-канал для теста.
func newServoTestLink(
	t *testing.T,
	version streamwriter.Version,
) (*TelemetrySource, *streamwriter.Writer, *frame.Reader) {
	t.Helper()
	return newSystemTestLink(t, version, 42)
}

func newSystemTestLink(
	t *testing.T,
	version streamwriter.Version,
	systemID uint8,
) (*TelemetrySource, *streamwriter.Writer, *frame.Reader) {
	t.Helper()
	port, err := net.ListenPacket("udp4", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	address := port.LocalAddr().String()
	if err := port.Close(); err != nil {
		t.Fatal(err)
	}
	source, err := NewTelemetrySource(address, systemID, slog.New(slog.DiscardHandler))
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

func TestServoFiltering(t *testing.T) {
	t.Parallel()
	source, writer, reader := newServoTestLink(t, streamwriter.V2)
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	send := func(systemID, componentID uint8, msg message.Message) {
		t.Helper()
		writer.SystemID, writer.ComponentID = systemID, componentID
		if err := writer.Write(msg); err != nil {
			t.Fatal(err)
		}
	}
	heartbeat := &ardupilotmega.MessageHeartbeat{Autopilot: ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA}
	outputs := &ardupilotmega.MessageServoOutputRaw{Port: 1, Servo1Raw: 1234, Servo16Raw: 1678}
	send(42, 1, outputs)
	send(42, 100, &ardupilotmega.MessageHeartbeat{Autopilot: ardupilotmega.MAV_AUTOPILOT_INVALID})
	send(42, 100, outputs)
	send(43, 1, heartbeat)
	send(43, 1, outputs)
	send(42, 1, heartbeat)
	event, err := source.Next(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if hb, ok := event.(model.Heartbeat); !ok || hb.SystemID != 42 || hb.ComponentID != 1 {
		t.Fatalf("telemetry before valid autopilot heartbeat was accepted: %+v", event)
	}

	for _, componentID := range []uint8{1, 2} {
		if componentID == 2 {
			send(42, componentID, heartbeat)
			event, err := source.Next(ctx)
			if err != nil {
				t.Fatal(err)
			}
			if hb, ok := event.(model.Heartbeat); !ok || hb.ComponentID != componentID {
				t.Fatalf("autopilot component was not updated: %+v", event)
			}
			send(42, 1, outputs)
		}
		requested := map[ardupilotmega.MAV_DATA_STREAM]bool{}
		for range 2 {
			packet, err := reader.Read()
			if err != nil {
				t.Fatalf("read stream request for component %d: %v", componentID, err)
			}
			request, ok := packet.GetMessage().(*ardupilotmega.MessageRequestDataStream)
			if !ok || request.TargetSystem != 42 || request.TargetComponent != componentID ||
				request.ReqMessageRate != 2 ||
				request.StartStop != 1 {
				t.Fatalf("unexpected stream request: %+v", packet.GetMessage())
			}
			requested[request.ReqStreamId] = true
		}
		if !requested[ardupilotmega.MAV_DATA_STREAM_POSITION] || !requested[ardupilotmega.MAV_DATA_STREAM_RC_CHANNELS] {
			t.Fatalf("incomplete stream requests for component %d: %v", componentID, requested)
		}
		send(43, componentID, outputs)
		send(42, 100, outputs)
		send(42, componentID, outputs)
		event, err := source.Next(ctx)
		if err != nil {
			t.Fatal(err)
		}
		got, ok := event.(model.ServoOutputs)
		if !ok || got.SystemID != 42 || got.ComponentID != componentID || got.Port != 1 || got.PWMUS[0] != 1234 ||
			got.PWMUS[15] != 1678 ||
			got.ReceivedAt.IsZero() {
			t.Fatalf("unexpected servo outputs after filtering: %+v", event)
		}
	}
}

func TestServoVersions(t *testing.T) {
	t.Parallel()
	values := ardupilotmega.MessageServoOutputRaw{Servo1Raw: 1100, Servo8Raw: 1800, Servo9Raw: 1900, Servo16Raw: 1600}
	for _, test := range []struct {
		name    string
		version streamwriter.Version
		output  ardupilotmega.MessageServoOutputRaw
		want    [16]uint16
	}{
		{"v1Values", streamwriter.V1, values, [16]uint16{0: 1100, 7: 1800}},
		{"v1Zero", streamwriter.V1, ardupilotmega.MessageServoOutputRaw{}, [16]uint16{}},
		{"v2Values", streamwriter.V2, values, [16]uint16{0: 1100, 7: 1800, 8: 1900, 15: 1600}},
		{"v2Zero", streamwriter.V2, ardupilotmega.MessageServoOutputRaw{}, [16]uint16{}},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			source, writer, _ := newServoTestLink(t, test.version)
			ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
			defer cancel()
			if err := writer.Write(&ardupilotmega.MessageHeartbeat{Autopilot: ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA}); err != nil {
				t.Fatal(err)
			}
			if _, err := source.Next(ctx); err != nil {
				t.Fatal(err)
			}
			if err := writer.Write(&test.output); err != nil {
				t.Fatal(err)
			}
			event, err := source.Next(ctx)
			if err != nil {
				t.Fatal(err)
			}
			got, ok := event.(model.ServoOutputs)
			if !ok || got.PWMUS != test.want || got.ReceivedAt.IsZero() {
				t.Fatalf("servo outputs = %+v, want %v", event, test.want)
			}
		})
	}
}
