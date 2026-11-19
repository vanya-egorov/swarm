package group

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/route"
)

// DefaultGroupRouteConfig задаёт одинаковый квадрат в раздельных областях радиусом 3,5 м.
func DefaultGroupRouteConfig() RouteCheckConfig {
	config := route.DefaultRouteCheckConfig()
	config.Flight.MaxDistanceM = 3.5
	return config
}

// RunGroupRouteCheck синхронизирует зависание и каждую точку маршрута всей группы.
func RunGroupRouteCheck(
	ctx context.Context, members []RouteMember, logger *slog.Logger,
	config RouteCheckConfig, minSeparationM float64,
) error {
	if err := config.Validate(); err != nil {
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
		targets[i], err = route.TargetsFrom(origin.Position, config.Waypoints)
		if err != nil {
			return fmt.Errorf("vehicle %d route: %w", origin.SystemID, err)
		}
	}
	return runGroupFlight(ctx, flights, logger, config.Flight, minSeparationM,
		func(ctx context.Context, before []State) error {
			hover := make([]PositionTarget, len(members))
			step := make([]PositionTarget, len(members))
			for i, state := range before {
				if !flight.SameDrone(origins[i], state) || coordinates.HorizontalDistance(origins[i].Position.LatitudeDeg,
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

// followGroupTargets продолжает удерживать цели быстрых участников до готовности всей группы.
func followGroupTargets(
	ctx context.Context, members []RouteMember, origins []State, config RouteCheckConfig, targets []PositionTarget,
) error {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	firstSent := make([]time.Time, len(members))
	arrived := make([]time.Time, len(members))
	var lastSent time.Time
	for {
		if err := ctx.Err(); err != nil {
			return context.Cause(ctx)
		}
		allArrived := true
		resend := lastSent.IsZero() || time.Since(lastSent) >= time.Second
		for i, member := range members {
			state := member.States.Snapshot()
			if err := flight.CheckState(state, origins[i], config.Flight, true); err != nil {
				return fmt.Errorf("vehicle %d target: %w", origins[i].SystemID, err)
			}
			if resend {
				sent := time.Now()
				if err := flight.RunCommand(ctx, config.Flight.CommandTimeout, func(ctx context.Context) error {
					return member.Positions.SetPositionTarget(ctx, targets[i])
				}); err != nil {
					return fmt.Errorf("vehicle %d target command: %w", origins[i].SystemID, err)
				}
				if firstSent[i].IsZero() {
					firstSent[i] = sent
				}
			}
			p := state.Position
			if p.ReceivedAt.After(firstSent[i]) && route.NearTarget(p, targets[i], config) {
				if arrived[i].IsZero() {
					arrived[i] = p.ReceivedAt
				}
			} else {
				arrived[i] = time.Time{}
			}
			if arrived[i].IsZero() || p.ReceivedAt.Sub(arrived[i]) < config.ArrivalDuration {
				allArrived = false
			}
		}
		if resend {
			lastSent = time.Now()
		}
		if allArrived {
			return nil
		}
		select {
		case <-ctx.Done():
			return context.Cause(ctx)
		case <-ticker.C:
		}
	}
}
