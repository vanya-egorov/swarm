package mavlink

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"net"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// ErrEventStreamClosed обозначает закрытие потока событий MAVLink.
var ErrEventStreamClosed = errors.New("MAVLink stream closed")

// TelemetrySource получает MAVLink по UDP без отправки полётных команд.
type TelemetrySource struct {
	node                 *gomavlib.Node
	expectedSystemID     uint8
	autopilotComponentID uint8
	requestedChannel     *gomavlib.Channel
	requestedComponentID uint8
	positionRequested    bool
	servoRequested       bool
	logger               *slog.Logger
	controller           *ArmingController
}

// NewTelemetrySource открывает UDP-приёмник для выбранного MAVLink system ID.
func NewTelemetrySource(listenAddress string, expectedSystemID uint8, logger *slog.Logger) (*TelemetrySource, error) {
	if logger == nil {
		return nil, errors.New("logger required")
	}
	if expectedSystemID == 0 {
		return nil, errors.New("system ID out of range")
	}
	if _, err := net.ResolveUDPAddr("udp4", listenAddress); err != nil {
		return nil, fmt.Errorf("UDP address %q: %w", listenAddress, err)
	}

	node := &gomavlib.Node{
		Endpoints:           []gomavlib.EndpointConf{gomavlib.EndpointUDPServer{Address: listenAddress}},
		Dialect:             ardupilotmega.Dialect,
		OutVersion:          gomavlib.V2,
		OutSystemID:         250,
		HeartbeatDisable:    true,
		StreamRequestEnable: false,
	}
	if err := node.Initialize(); err != nil {
		return nil, fmt.Errorf("listen %s: %w", listenAddress, err)
	}

	return &TelemetrySource{node: node, expectedSystemID: expectedSystemID, logger: logger}, nil
}

// Close освобождает UDP-приёмник и связанные ресурсы.
func (s *TelemetrySource) Close() {
	if s.controller != nil {
		s.controller.close()
	}
	s.node.Close()
}

// Next ожидает heartbeat, позицию или PWM-выходы нужного автопилота либо отмену контекста.
func (s *TelemetrySource) Next(ctx context.Context) (vehicle.Telemetry, error) {
	for {
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case event, ok := <-s.node.Events():
			if !ok {
				return nil, ErrEventStreamClosed
			}

			switch event := event.(type) {
			case *gomavlib.EventFrame:
				if telemetry := s.frameTelemetry(event); telemetry != nil {
					return telemetry, nil
				}
			case *gomavlib.EventParseError:
				s.logger.Warn("invalid frame", "error", event.Error)
			case *gomavlib.EventChannelClose:
				if s.controller != nil {
					s.controller.channelClosed(event.Channel)
				}
				s.logger.Warn("channel closed", "error", event.Error)
			}
		}
	}
}

// requestStream запрашивает группу телеметрии и сообщает об успешной отправке.
func (s *TelemetrySource) requestStream(event *gomavlib.EventFrame, stream common.MAV_DATA_STREAM, name string) bool {
	err := s.node.WriteMessageTo(event.Channel, &ardupilotmega.MessageRequestDataStream{
		TargetSystem: event.SystemID(), TargetComponent: event.ComponentID(),
		ReqStreamId: stream, ReqMessageRate: 2, StartStop: 1,
	})
	if err != nil {
		s.logger.Warn("stream request failed", "stream", name, "systemId", event.SystemID(), "error", err)
		return false
	}
	s.logger.Info("stream requested", "stream", name, "systemId", event.SystemID(), "rateHz", 2)
	return true
}
