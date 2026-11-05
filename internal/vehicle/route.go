package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
)

// RunRouteCheck проверяет весь маршрут до arm и сохраняет посадку при любой ошибке.
func RunRouteCheck(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	positions PositionController,
	states StateReader,
	logger *slog.Logger,
	config RouteCheckConfig,
) error {
	if positions == nil || flight == nil || arming == nil || states == nil || logger == nil {
		return errors.New("route check: missing dependency")
	}
	if err := config.validate(); err != nil {
		return fmt.Errorf("route config: %w", err)
	}
	readyCtx, cancel := context.WithTimeout(ctx, config.Flight.ReadyTimeout)
	origin, err := waitForState(
		readyCtx,
		states,
		func(s State) bool { return freshState(s, config.Flight.TelemetryMaxAge) },
	)
	cancel()
	if err != nil {
		return fmt.Errorf("route origin: %w", err)
	}
	targets, err := routeTargets(origin.Position, config.Waypoints)
	if err != nil {
		return err
	}
	return runFlightCheck(ctx, flight, arming, states, logger, config.Flight, func(before State) error {
		// Место старта не должно измениться между подготовкой и взлётом.
		if !sameVehicle(origin, before) ||
			horizontalDistance(
				origin.Position.LatitudeDeg,
				origin.Position.LongitudeDeg,
				before.Position.LatitudeDeg,
				before.Position.LongitudeDeg,
			) > config.HorizontalToleranceM {
			return errors.New("route origin changed")
		}
		// Горизонтальное движение начинается после стабилизации подъёма.
		settleCtx, cancel := context.WithTimeout(ctx, config.Flight.ClimbTimeout)
		err := followTarget(settleCtx, positions, states, before, config, PositionTarget{
			LatitudeDeg:       before.Position.LatitudeDeg,
			LongitudeDeg:      before.Position.LongitudeDeg,
			RelativeAltitudeM: config.Flight.TargetAltitudeM,
		})
		cancel()
		if err != nil {
			return fmt.Errorf("hover: %w", err)
		}
		logger.Info("hover stable", "systemId", before.SystemID)
		for i, target := range targets {
			logger.Info("waypoint started", "index", i+1, "target", target)
			waypointCtx, cancel := context.WithTimeout(ctx, config.WaypointTimeout)
			err := followTarget(waypointCtx, positions, states, before, config, target)
			cancel()
			if err != nil {
				return fmt.Errorf("waypoint %d: %w", i+1, err)
			}
			logger.Info("waypoint reached", "index", i+1)
		}
		return nil
	})
}
