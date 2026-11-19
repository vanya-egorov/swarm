package route

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	flightcore "github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type State = telemetry.State
type StateReader = telemetry.StateReader
type Position = telemetry.Position
type FlightMode = telemetry.FlightMode
type FlightCheckConfig = flightcore.FlightCheckConfig
type FlightController = flightcore.FlightController
type ArmingController = flightcore.ArmingController

const (
	ModeStabilize = telemetry.ModeStabilize
	ModeGuided    = telemetry.ModeGuided
	ModeLand      = telemetry.ModeLand
)

// Waypoint is a north/east offset from the launch point and an altitude above home.
type Waypoint struct{ NorthM, EastM, AltitudeM float64 }

// PositionTarget contains WGS84 coordinates and altitude above home.
type PositionTarget struct{ LatitudeDeg, LongitudeDeg, RelativeAltitudeM float64 }

// PositionController sends a target; telemetry confirms arrival.
type PositionController interface {
	SetPositionTarget(context.Context, PositionTarget) error
}

// VelocityController задаёт скорость north/east/down в GUIDED, в м/с.
type VelocityController interface {
	SetVelocityTarget(context.Context, float64, float64, float64) error
}

// NavigationSpeedController подтверждает ограничение горизонтальной скорости автопилотом.
type NavigationSpeedController interface {
	SetNavigationSpeed(context.Context, float64) error
}

// TargetFollower заменяет прямой перелёт локальным навигатором с проверкой сенсоров.
type TargetFollower interface {
	FollowTarget(context.Context, State, RouteCheckConfig, PositionTarget) error
	HoldTarget(context.Context, State, RouteCheckConfig, PositionTarget) error
}

// StageCoordinator разрешает этапы и сохраняет подтверждённый прогресс маршрута.
// Этап 1 — взлёт и зависание; затем точки; последний барьер разрешает посадку.
type StageCoordinator interface {
	Wait(context.Context, int) error
	Reached(int) error
}

// StageFailureObserver позволяет сообщить об отказе задания до начала посадки.
type StageFailureObserver interface{ Failed(error) }

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
	return runRouteCheck(ctx, flight, arming, positions, states, logger, config, nil)
}

// RunCoordinatedRouteCheck исполняет маршрут с внешними барьерами и штатной посадкой при отказе.
func RunCoordinatedRouteCheck(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	positions PositionController,
	states StateReader,
	logger *slog.Logger,
	config RouteCheckConfig,
	coordinator StageCoordinator,
) error {
	if coordinator == nil {
		return errors.New("route coordinator required")
	}
	return runRouteCheck(ctx, flight, arming, positions, states, logger, config, coordinator)
}

func runRouteCheck(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	positions PositionController,
	states StateReader,
	logger *slog.Logger,
	config RouteCheckConfig,
	coordinator StageCoordinator,
) error {
	if positions == nil || flight == nil || arming == nil || states == nil || logger == nil {
		return errors.New("route check: missing dependency")
	}
	if err := config.Validate(); err != nil {
		return fmt.Errorf("route config: %w", err)
	}
	readyCtx, cancel := context.WithTimeout(ctx, config.Flight.ReadyTimeout)
	origin, err := flightcore.WaitForState(
		readyCtx,
		states,
		func(s State) bool { return flightcore.FreshState(s, config.Flight.TelemetryMaxAge) },
	)
	cancel()
	if err != nil {
		return fmt.Errorf("route origin: %w", err)
	}
	targets, err := TargetsFrom(origin.Position, config.Waypoints)
	if err != nil {
		return err
	}
	if coordinator != nil {
		if err := coordinator.Wait(ctx, 1); err != nil {
			return fmt.Errorf("launch barrier: %w", err)
		}
	}
	return flightcore.RunFlightCheckTask(
		ctx,
		flight,
		arming,
		states,
		logger,
		config.Flight,
		func(before State) (taskErr error) {
			defer func() {
				if observer, ok := coordinator.(StageFailureObserver); ok && taskErr != nil {
					observer.Failed(taskErr)
				}
			}()
			// Место старта не должно измениться между подготовкой и взлётом.
			if !flightcore.SameDrone(origin, before) ||
				coordinates.HorizontalDistance(
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
			current := PositionTarget{LatitudeDeg: before.Position.LatitudeDeg,
				LongitudeDeg: before.Position.LongitudeDeg, RelativeAltitudeM: config.Flight.TargetAltitudeM}
			if coordinator != nil {
				if err := coordinator.Reached(1); err != nil {
					return err
				}
			}
			for i, target := range targets {
				if err := waitForStage(ctx, coordinator, i+2, positions, states, before, config, current); err != nil {
					return err
				}
				logger.Info("waypoint started", "index", i+1, "target", target)
				waypointCtx, cancel := context.WithTimeout(ctx, config.WaypointTimeout)
				err := followTarget(waypointCtx, positions, states, before, config, target)
				cancel()
				if err != nil {
					return fmt.Errorf("waypoint %d: %w", i+1, err)
				}
				logger.Info("waypoint reached", "index", i+1)
				current = target
				if coordinator != nil {
					if err := coordinator.Reached(i + 2); err != nil {
						return err
					}
				}
			}
			return waitForStage(ctx, coordinator, len(targets)+2, positions, states, before, config, current)
		},
	)
}

func waitForStage(ctx context.Context, coordinator StageCoordinator, stage int, positions PositionController,
	states StateReader, before State, config RouteCheckConfig, target PositionTarget) error {
	if coordinator == nil {
		return nil
	}
	waitCtx, cancel := context.WithCancel(ctx)
	done := make(chan error, 1)
	go func() { done <- coordinator.Wait(waitCtx, stage) }()
	defer cancel()
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	lastSent := time.Now()
	for {
		select {
		case err := <-done:
			return err
		case <-ctx.Done():
			cancel()
			<-done
			return ctx.Err()
		case <-ticker.C:
			if err := flightcore.CheckState(states.Snapshot(), before, config.Flight, true); err != nil {
				cancel()
				<-done
				return err
			}
			if navigator, ok := positions.(TargetFollower); ok {
				if err := navigator.HoldTarget(ctx, before, config, target); err != nil {
					cancel()
					<-done
					return err
				}
				continue
			}
			if time.Since(lastSent) >= time.Second {
				if err := flightcore.RunCommand(ctx, config.Flight.CommandTimeout,
					func(c context.Context) error { return positions.SetPositionTarget(c, target) }); err != nil {
					cancel()
					<-done
					return err
				}
				lastSent = time.Now()
			}
		}
	}
}

func followTarget(
	ctx context.Context,
	positions PositionController,
	states StateReader,
	before State,
	config RouteCheckConfig,
	target PositionTarget,
) error {
	if navigator, ok := positions.(TargetFollower); ok {
		return navigator.FollowTarget(ctx, before, config, target)
	}
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	var firstSent, lastSent, arrived time.Time
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		state := states.Snapshot()
		if err := flightcore.CheckState(state, before, config.Flight, true); err != nil {
			return err
		}
		if lastSent.IsZero() || time.Since(lastSent) >= time.Second {
			sent := time.Now()
			if err := flightcore.RunCommand(ctx, config.Flight.CommandTimeout, func(c context.Context) error { return positions.SetPositionTarget(c, target) }); err != nil {
				return err
			}
			lastSent = sent
			if firstSent.IsZero() {
				firstSent = sent
			}
		}
		p := state.Position
		atTarget := p.ReceivedAt.After(firstSent) && NearTarget(p, target, config)
		if atTarget {
			if arrived.IsZero() {
				arrived = p.ReceivedAt
			}
			if p.ReceivedAt.Sub(arrived) >= config.ArrivalDuration {
				return nil
			}
		} else {
			arrived = time.Time{}
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

func NearTarget(p *Position, target PositionTarget, config RouteCheckConfig) bool {
	return coordinates.HorizontalDistance(
		p.LatitudeDeg,
		p.LongitudeDeg,
		target.LatitudeDeg,
		target.LongitudeDeg,
	) <= config.HorizontalToleranceM &&
		math.Abs(p.RelativeAltitudeM-target.RelativeAltitudeM) <= config.AltitudeToleranceM &&
		math.Hypot(p.NorthVelocityMS, p.EastVelocityMS) < 0.3 &&
		math.Abs(p.DownVelocityMS) < 0.3
}

// Validate проверяет координаты и допустимую высоту цели SITL.
func (p PositionTarget) Validate() error {
	if !coordinates.Finite(p.LatitudeDeg) || math.Abs(p.LatitudeDeg) > 90 || !coordinates.Finite(p.LongitudeDeg) ||
		math.Abs(p.LongitudeDeg) > 180 ||
		!coordinates.Finite(p.RelativeAltitudeM) ||
		p.RelativeAltitudeM < 0.8 ||
		p.RelativeAltitudeM > 2 {
		return errors.New("invalid position target")
	}
	return nil
}

func TargetsFrom(origin *telemetry.Position, waypoints []Waypoint) ([]PositionTarget, error) {
	if !coordinates.ValidPosition(origin) || math.Abs(origin.LatitudeDeg) > 85 {
		return nil, errors.New("invalid route origin (latitude must be within ±85 degrees)")
	}
	targets := make([]PositionTarget, len(waypoints))
	for i, p := range waypoints {
		lon := origin.LongitudeDeg + p.EastM/(coordinates.MetersPerDegree*math.Cos(origin.LatitudeDeg*math.Pi/180))
		lon = math.Mod(lon+540, 360) - 180
		targets[i] = PositionTarget{origin.LatitudeDeg + p.NorthM/coordinates.MetersPerDegree, lon, p.AltitudeM}
		if err := targets[i].Validate(); err != nil {
			return nil, err
		}
	}
	return targets, nil
}
