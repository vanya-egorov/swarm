package telemetry

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"net"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"

	model "github.com/vanya-egorov/swarm/internal/drone/telemetry"
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
	observer             EventObserver
}

// EventObserver receives command-relevant events from the source's single reader.
type EventObserver interface {
	ObserveHeartbeat(*gomavlib.Channel, model.Heartbeat)
	ObserveACK(*gomavlib.EventFrame, *ardupilotmega.MessageCommandAck)
	ChannelClosed(*gomavlib.Channel)
	Close()
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
	if s.observer != nil {
		s.observer.Close()
	}
	s.node.Close()
}

// AttachObserver installs the optional command event observer before reading starts.
func (s *TelemetrySource) AttachObserver(observer EventObserver) error {
	if observer == nil {
		return errors.New("observer required")
	}
	if s.observer != nil {
		return errors.New("observer already attached")
	}
	s.observer = observer
	return nil
}

// ExpectedSystemID returns the system ID selected when the source was created.
func (s *TelemetrySource) ExpectedSystemID() uint8 { return s.expectedSystemID }

// OutboundIDs returns the MAVLink source IDs used for outgoing messages.
func (s *TelemetrySource) OutboundIDs() (systemID, componentID uint8) {
	return s.node.OutSystemID, s.node.OutComponentID
}

// Logger returns the logger configured for the source.
func (s *TelemetrySource) Logger() *slog.Logger { return s.logger }

// WriteMessageTo sends a MAVLink message through a channel owned by this source.
func (s *TelemetrySource) WriteMessageTo(channel *gomavlib.Channel, msg message.Message) error {
	return s.node.WriteMessageTo(channel, msg)
}

// Next ожидает heartbeat, позицию или PWM-выходы нужного автопилота либо отмену контекста.
func (s *TelemetrySource) Next(ctx context.Context) (model.Telemetry, error) {
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
				if s.observer != nil {
					s.observer.ChannelClosed(event.Channel)
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
