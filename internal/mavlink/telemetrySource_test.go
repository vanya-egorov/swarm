package mavlink

import (
	"context"
	"log/slog"
	"net"
	"testing"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

func TestTelemetrySource(t *testing.T) {
	t.Parallel()
	port, err := net.ListenPacket("udp4", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	address := port.LocalAddr().String()
	if err := port.Close(); err != nil {
		t.Fatal(err)
	}

	source, err := NewTelemetrySource(address, 42, slog.Default())
	if err != nil {
		t.Fatal(err)
	}
	defer source.Close()

	sender := &gomavlib.Node{
		Endpoints:        []gomavlib.EndpointConf{gomavlib.EndpointUDPClient{Address: address}},
		Dialect:          ardupilotmega.Dialect,
		OutVersion:       gomavlib.V2,
		OutSystemID:      42,
		HeartbeatDisable: true,
	}
	if err := sender.Initialize(); err != nil {
		t.Fatal(err)
	}
	defer sender.Close()
	otherSender := &gomavlib.Node{
		Endpoints:        []gomavlib.EndpointConf{gomavlib.EndpointUDPClient{Address: address}},
		Dialect:          ardupilotmega.Dialect,
		OutVersion:       gomavlib.V2,
		OutSystemID:      43,
		HeartbeatDisable: true,
	}
	if err := otherSender.Initialize(); err != nil {
		t.Fatal(err)
	}
	defer otherSender.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	stopSending := make(chan struct{})
	defer close(stopSending)
	go func() {
		ticker := time.NewTicker(30 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-stopSending:
				return
			case <-ticker.C:
				_ = otherSender.WriteMessageAll(&ardupilotmega.MessageHeartbeat{
					Type:      ardupilotmega.MAV_TYPE_QUADROTOR,
					Autopilot: ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA,
				})
				_ = otherSender.WriteMessageAll(&ardupilotmega.MessageGlobalPositionInt{Lat: 100000000, Lon: 200000000})
				_ = otherSender.WriteMessageAll(&ardupilotmega.MessageServoOutputRaw{Servo1Raw: 1800})
				_ = sender.WriteMessageAll(&ardupilotmega.MessageHeartbeat{
					Type:      ardupilotmega.MAV_TYPE_QUADROTOR,
					Autopilot: ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA,
					BaseMode:  ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED,
				})
				_ = sender.WriteMessageAll(&ardupilotmega.MessageGlobalPositionInt{
					TimeBootMs:  1200,
					Lat:         557500000,
					Lon:         376200000,
					Alt:         180500,
					RelativeAlt: 12500,
					Vx:          250,
					Vy:          -75,
					Vz:          10,
					Hdg:         12345,
				})
				_ = sender.WriteMessageAll(&ardupilotmega.MessageGlobalPositionInt{
					Lat: 557500000,
					Lon: 376200000,
					Hdg: 65535,
				})
				_ = sender.WriteMessageAll(&ardupilotmega.MessageServoOutputRaw{
					TimeUsec:   1200000,
					Port:       0,
					Servo1Raw:  1100,
					Servo2Raw:  1200,
					Servo3Raw:  1300,
					Servo4Raw:  1400,
					Servo5Raw:  1500,
					Servo6Raw:  1600,
					Servo7Raw:  1700,
					Servo8Raw:  1800,
					Servo9Raw:  1900,
					Servo10Raw: 2000,
					Servo11Raw: 1101,
					Servo12Raw: 1201,
					Servo13Raw: 1301,
					Servo14Raw: 1401,
					Servo15Raw: 1501,
					Servo16Raw: 1601,
				})
			}
		}
	}()

	first, err := source.Next(ctx)
	if err != nil {
		t.Fatal(err)
	}
	heartbeat, ok := first.(vehicle.Heartbeat)
	if !ok {
		t.Fatalf("first telemetry = %T, want heartbeat", first)
	}
	if heartbeat.SystemID != 42 || heartbeat.ComponentID != 1 || !heartbeat.Armed {
		t.Fatalf("unexpected heartbeat: %+v", heartbeat)
	}
	if heartbeat.ReceivedAt.IsZero() {
		t.Fatal("heartbeat reception time is missing")
	}

	requested := map[ardupilotmega.MAV_DATA_STREAM]bool{}
	for len(requested) < 2 {
		select {
		case <-ctx.Done():
			t.Fatalf("telemetry stream requests incomplete: received %v", requested)
		case event := <-sender.Events():
			frame, ok := event.(*gomavlib.EventFrame)
			if !ok {
				continue
			}
			request, ok := frame.Message().(*ardupilotmega.MessageRequestDataStream)
			if !ok {
				continue
			}
			if request.TargetSystem != 42 || request.TargetComponent != 1 || request.ReqMessageRate != 2 || request.StartStop != 1 {
				t.Fatalf("unexpected telemetry stream request: %+v", request)
			}
			if request.ReqStreamId != ardupilotmega.MAV_DATA_STREAM_POSITION && request.ReqStreamId != ardupilotmega.MAV_DATA_STREAM_RC_CHANNELS {
				t.Fatalf("unexpected stream ID: %v", request.ReqStreamId)
			}
			requested[request.ReqStreamId] = true
		}
	}

	knownHeading := false
	unknownHeading := false
	servoSeen := false
	for !knownHeading || !unknownHeading || !servoSeen {
		event, err := source.Next(ctx)
		if err != nil {
			t.Fatal(err)
		}
		if outputs, ok := event.(vehicle.ServoOutputs); ok {
			wantPWM := [16]uint16{1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 1101, 1201, 1301, 1401, 1501, 1601}
			if outputs.SystemID != 42 || outputs.ComponentID != 1 || outputs.Port != 0 || outputs.PWMUS != wantPWM || outputs.ReceivedAt.IsZero() {
				t.Fatalf("unexpected servo outputs: %+v", outputs)
			}
			servoSeen = true
			continue
		}
		position, ok := event.(vehicle.Position)
		if !ok {
			continue
		}
		if position.SystemID != 42 || position.ComponentID != 1 || position.LatitudeDeg != 55.75 || position.LongitudeDeg != 37.62 {
			t.Fatalf("unexpected position coordinates: %+v", position)
		}
		if position.HeadingValid {
			if position.AltitudeMSLM != 180.5 || position.RelativeAltitudeM != 12.5 || position.NorthVelocityMS != 2.5 || position.EastVelocityMS != -0.75 || position.DownVelocityMS != 0.1 {
				t.Fatalf("unexpected position units: %+v", position)
			}
			if position.HeadingDeg != 123.45 || position.BootTime != 1200*time.Millisecond || position.ReceivedAt.IsZero() {
				t.Fatalf("unexpected position metadata: %+v", position)
			}
			knownHeading = true
		} else {
			if position.HeadingDeg != 0 {
				t.Fatalf("unknown heading must not have a numeric value: %+v", position)
			}
			unknownHeading = true
		}
	}
}

func TestSourceConfig(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name, address string
		systemID      uint8
		logger        *slog.Logger
	}{
		{"address", "bad-address", 1, slog.Default()},
		{"systemId", "127.0.0.1:14560", 0, slog.Default()},
		{"logger", "127.0.0.1:14560", 1, nil},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			if _, err := NewTelemetrySource(test.address, test.systemID, test.logger); err == nil {
				t.Fatal("invalid config accepted")
			}
		})
	}
}
