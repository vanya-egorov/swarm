package mavlink

import (
	"context"
	"fmt"
	"math"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"

	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// SetMode переводит локальный ArduCopter в GUIDED и подтверждает режим heartbeat.
func (c *ArmingController) SetMode(ctx context.Context, mode vehicle.FlightMode) error {
	if mode != vehicle.ModeGuided {
		return fmt.Errorf("unsupported mode %q", mode)
	}
	return c.execute(ctx, common.MAV_CMD_DO_SET_MODE, func(p *pendingCommand, _ int) error {
		return c.source.node.WriteMessageTo(p.channel, &ardupilotmega.MessageCommandInt{
			TargetSystem: c.source.expectedSystemID, TargetComponent: p.componentID,
			Frame: common.MAV_FRAME_MISSION, Command: common.MAV_CMD_DO_SET_MODE,
			Param1: float32(
				common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
			), Param2: float32(ardupilotmega.COPTER_MODE_GUIDED),
		})
	}, func(hb vehicle.Heartbeat) bool { return hb.Mode == mode })
}

// Takeoff запрашивает высоту относительно точки старта и ожидает принятия команды.
func (c *ArmingController) Takeoff(ctx context.Context, altitudeM float64) error {
	if altitudeM < 0.8 || altitudeM > 2 || altitudeM != altitudeM {
		return fmt.Errorf("takeoff altitude out of range: %g m", altitudeM)
	}
	return c.execute(ctx, common.MAV_CMD_NAV_TAKEOFF, func(p *pendingCommand, _ int) error {
		return c.source.node.WriteMessageTo(p.channel, &ardupilotmega.MessageCommandInt{
			TargetSystem: c.source.expectedSystemID, TargetComponent: p.componentID,
			Frame: common.MAV_FRAME_GLOBAL_RELATIVE_ALT, Command: common.MAV_CMD_NAV_TAKEOFF,
			Z: float32(altitudeM),
		})
	}, nil, "altitudeM", altitudeM)
}

// Land включает штатную посадку и подтверждает переход в LAND heartbeat.
func (c *ArmingController) Land(ctx context.Context) error {
	return c.execute(ctx, common.MAV_CMD_NAV_LAND, func(p *pendingCommand, _ int) error {
		return c.source.node.WriteMessageTo(p.channel, &ardupilotmega.MessageCommandInt{
			TargetSystem: c.source.expectedSystemID, TargetComponent: p.componentID,
			Frame: common.MAV_FRAME_GLOBAL_RELATIVE_ALT, Command: common.MAV_CMD_NAV_LAND,
		})
	}, func(hb vehicle.Heartbeat) bool { return hb.Mode == vehicle.ModeLand })
}

// SetPositionTarget отправляет только позицию в GUIDED; ACK этим сообщением не предусмотрен.
func (c *ArmingController) SetPositionTarget(ctx context.Context, target vehicle.PositionTarget) error {
	if err := target.Validate(); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed || c.channel == nil {
		return vehicle.ErrCommandLinkChanged
	}
	if c.pending != nil {
		return vehicle.ErrCommandBusy
	}
	age := time.Since(c.heartbeat.ReceivedAt)
	if age < 0 || age > c.config.HeartbeatMaxAge {
		return vehicle.ErrHeartbeatTimeout
	}
	if !c.heartbeat.Armed || c.heartbeat.Mode != vehicle.ModeGuided {
		return fmt.Errorf("position target requires armed GUIDED")
	}
	return c.source.node.WriteMessageTo(c.channel, &ardupilotmega.MessageSetPositionTargetGlobalInt{
		TargetSystem: c.source.expectedSystemID, TargetComponent: c.heartbeat.ComponentID,
		CoordinateFrame: common.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		TypeMask: common.POSITION_TARGET_TYPEMASK_VX_IGNORE | common.POSITION_TARGET_TYPEMASK_VY_IGNORE | common.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
			common.POSITION_TARGET_TYPEMASK_AX_IGNORE | common.POSITION_TARGET_TYPEMASK_AY_IGNORE | common.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
			common.POSITION_TARGET_TYPEMASK_YAW_IGNORE | common.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
		LatInt: int32(
			math.Round(target.LatitudeDeg * 1e7),
		), LonInt: int32(math.Round(target.LongitudeDeg * 1e7)), Alt: float32(target.RelativeAltitudeM),
	})
}
