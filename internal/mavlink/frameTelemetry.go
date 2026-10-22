package mavlink

import (
	"strings"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// frameTelemetry принимает данные только выбранной системы и компонента.
func (s *TelemetrySource) frameTelemetry(frame *gomavlib.EventFrame) vehicle.Telemetry {
	if frame.SystemID() != s.expectedSystemID {
		return nil
	}
	switch msg := frame.Message().(type) {
	case *ardupilotmega.MessageHeartbeat:
		if msg.Autopilot != ardupilotmega.MAV_AUTOPILOT_ARDUPILOTMEGA {
			return nil
		}
		return s.heartbeat(frame, msg)
	case *ardupilotmega.MessageCommandAck:
		if s.controller != nil {
			s.controller.observeACK(frame, msg)
		}
	case *ardupilotmega.MessageStatustext:
		if s.fromAutopilot(frame) {
			s.logger.Info("autopilot status", "systemId", frame.SystemID(), "componentId", frame.ComponentID(), "severity", msg.Severity.String(), "text", strings.TrimRight(msg.Text, "\x00"))
		}
	case *ardupilotmega.MessageGlobalPositionInt:
		if s.fromAutopilot(frame) {
			return decodePosition(frame, msg)
		}
	case *ardupilotmega.MessageServoOutputRaw:
		if s.fromAutopilot(frame) {
			return decodeServo(frame, msg)
		}
	}
	return nil
}

// fromAutopilot проверяет компонент автопилота после первого heartbeat.
func (s *TelemetrySource) fromAutopilot(frame *gomavlib.EventFrame) bool {
	return s.autopilotComponentID != 0 && frame.ComponentID() == s.autopilotComponentID
}

// heartbeat обновляет компонент, запрашивает потоки и декодирует режим.
func (s *TelemetrySource) heartbeat(frame *gomavlib.EventFrame, msg *ardupilotmega.MessageHeartbeat) vehicle.Heartbeat {
	s.autopilotComponentID = frame.ComponentID()
	if s.requestedChannel != frame.Channel || s.requestedComponentID != frame.ComponentID() {
		s.requestedChannel, s.requestedComponentID = frame.Channel, frame.ComponentID()
		s.positionRequested, s.servoRequested = false, false
	}
	if !s.positionRequested {
		s.positionRequested = s.requestStream(frame, ardupilotmega.MAV_DATA_STREAM_POSITION, "position")
	}
	if !s.servoRequested {
		s.servoRequested = s.requestStream(frame, ardupilotmega.MAV_DATA_STREAM_RC_CHANNELS, "servo")
	}
	heartbeat := vehicle.Heartbeat{
		SystemID:    frame.SystemID(),
		ComponentID: frame.ComponentID(),
		Armed:       msg.BaseMode&ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED != 0,
		Mode:        vehicle.ModeUnknown,
		ReceivedAt:  time.Now().UTC(),
	}
	if msg.Type == ardupilotmega.MAV_TYPE_QUADROTOR && msg.BaseMode&ardupilotmega.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED != 0 {
		switch msg.CustomMode {
		case uint32(ardupilotmega.COPTER_MODE_STABILIZE):
			heartbeat.Mode = vehicle.ModeStabilize
		case uint32(ardupilotmega.COPTER_MODE_GUIDED):
			heartbeat.Mode = vehicle.ModeGuided
		case uint32(ardupilotmega.COPTER_MODE_LAND):
			heartbeat.Mode = vehicle.ModeLand
		}
	}
	if s.controller != nil {
		s.controller.observeHeartbeat(frame.Channel, heartbeat)
	}
	return heartbeat
}

// decodePosition переводит единицы MAVLink в модель дрона.
func decodePosition(frame *gomavlib.EventFrame, msg *ardupilotmega.MessageGlobalPositionInt) vehicle.Position {
	position := vehicle.Position{
		SystemID:          frame.SystemID(),
		ComponentID:       frame.ComponentID(),
		LatitudeDeg:       float64(msg.Lat) / 1e7,
		LongitudeDeg:      float64(msg.Lon) / 1e7,
		AltitudeMSLM:      float64(msg.Alt) / 1e3,
		RelativeAltitudeM: float64(msg.RelativeAlt) / 1e3,
		NorthVelocityMS:   float64(msg.Vx) / 1e2,
		EastVelocityMS:    float64(msg.Vy) / 1e2,
		DownVelocityMS:    float64(msg.Vz) / 1e2,
		BootTime:          time.Duration(msg.TimeBootMs) * time.Millisecond,
		ReceivedAt:        time.Now().UTC(),
	}
	if msg.Hdg != 65535 {
		position.HeadingDeg, position.HeadingValid = float64(msg.Hdg)/1e2, true
	}
	return position
}

// decodeServo сохраняет все 16 сырых PWM-выходов без преобразования.
func decodeServo(frame *gomavlib.EventFrame, msg *ardupilotmega.MessageServoOutputRaw) vehicle.ServoOutputs {
	return vehicle.ServoOutputs{
		SystemID:    frame.SystemID(),
		ComponentID: frame.ComponentID(),
		Port:        msg.Port,
		PWMUS: [16]uint16{
			msg.Servo1Raw, msg.Servo2Raw, msg.Servo3Raw, msg.Servo4Raw,
			msg.Servo5Raw, msg.Servo6Raw, msg.Servo7Raw, msg.Servo8Raw,
			msg.Servo9Raw, msg.Servo10Raw, msg.Servo11Raw, msg.Servo12Raw,
			msg.Servo13Raw, msg.Servo14Raw, msg.Servo15Raw, msg.Servo16Raw,
		},
		ReceivedAt: time.Now().UTC(),
	}
}
