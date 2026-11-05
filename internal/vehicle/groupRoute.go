package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
)

// RouteMember добавляет позиционные цели к управлению участником группы.
type RouteMember struct {
	FlightMember
	Positions PositionController
}

// DefaultGroupRouteConfig задаёт одинаковый квадрат в раздельных областях радиусом 3,5 м.
func DefaultGroupRouteConfig() RouteCheckConfig {
	config := DefaultRouteCheckConfig()
	config.Flight.MaxDistanceM = 3.5
	return config
}

// RunGroupRouteCheck синхронизирует зависание и каждую точку маршрута всей группы.
func RunGroupRouteCheck(
	ctx context.Context, members []RouteMember, logger *slog.Logger,
	config RouteCheckConfig, minSeparationM float64,
) error {
	if err := config.validate(); err != nil {
		return fmt.Errorf("group route config: %w", err)
	}
	flights := make([]FlightMember, len(members))
	for i, member := range members {
		if member.Positions == nil {
			return errors.New("group route: position controller required")
		}
		flights[i] = member.FlightMember
	}
	// Проверяем перевод всех координат до любых полётных команд, включая GUIDED.
	readyCtx, stop := context.WithTimeout(ctx, config.Flight.ReadyTimeout)
	for _, member := range flights {
		if member.States == nil {
			stop()
			return errors.New("group route: state reader required")
		}
	}
	origins, err := waitForGroup(readyCtx, flights, config.Flight.TelemetryMaxAge)
	stop()
	if err != nil {
		return err
	}
	targets := make([][]PositionTarget, len(members))
	for i, origin := range origins {
		targets[i], err = routeTargets(origin.Position, config.Waypoints)
		if err != nil {
			return fmt.Errorf("vehicle %d route: %w", origin.SystemID, err)
		}
	}
	return runGroupFlight(ctx, flights, logger, config.Flight, minSeparationM,
		func(ctx context.Context, before []State) error {
			hover := make([]PositionTarget, len(members))
			for i, state := range before {
				if !sameVehicle(origins[i], state) || horizontalDistance(origins[i].Position.LatitudeDeg,
					origins[i].Position.LongitudeDeg, state.Position.LatitudeDeg,
					state.Position.LongitudeDeg) > config.HorizontalToleranceM {
					return errors.New("group route origin changed")
				}
				hover[i] = PositionTarget{
					state.Position.LatitudeDeg,
					state.Position.LongitudeDeg,
					config.Flight.TargetAltitudeM,
				}
			}
			hoverCtx, cancel := context.WithTimeout(ctx, config.Flight.ClimbTimeout)
			err := followGroupTargets(hoverCtx, members, before, config, hover)
			cancel()
			if err != nil {
				return fmt.Errorf("group hover: %w", err)
			}
			logger.Info("group hover stable", "vehicles", len(members))
			for waypoint := range config.Waypoints {
				step := make([]PositionTarget, len(members))
				for i := range members {
					step[i] = targets[i][waypoint]
				}
				logger.Info("group waypoint started", "index", waypoint+1)
				waypointCtx, cancel := context.WithTimeout(ctx, config.WaypointTimeout)
				err := followGroupTargets(waypointCtx, members, before, config, step)
				cancel()
				if err != nil {
					return fmt.Errorf("group waypoint %d: %w", waypoint+1, err)
				}
				for _, state := range before {
					logger.Info("waypoint reached", "systemId", state.SystemID, "index", waypoint+1)
				}
				logger.Info("group waypoint reached", "index", waypoint+1, "vehicles", len(members))
			}
			return nil
		})
}
