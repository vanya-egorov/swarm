package cli

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"net"
	"strconv"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/group"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
	mavcommand "github.com/vanya-egorov/swarm/internal/mavlink/command"
	mavtelemetry "github.com/vanya-egorov/swarm/internal/mavlink/telemetry"
)

// runGroupCheck держит все три приёмника активными до завершения посадки группы.
func runGroupCheck(
	ctx context.Context,
	address string,
	systemID int,
	timeout time.Duration,
	logger *slog.Logger,
	route bool,
) error {
	host, portText, err := net.SplitHostPort(address)
	if err != nil {
		return err
	}
	port, err := strconv.Atoi(portText)
	if err != nil || port < 1024 || port > 65515 || systemID > 253 {
		return errors.New("invalid group IDs or ports")
	}
	members := make([]group.FlightMember, 0, 3)
	routeMembers := make([]group.RouteMember, 0, 3)
	monitors := make([]*telemetry.Monitor, 0, 3)
	for i := range 3 {
		source, err := mavtelemetry.NewTelemetrySource(
			net.JoinHostPort(host, strconv.Itoa(port+10*i)),
			uint8(systemID+i),
			logger,
		)
		if err != nil {
			return err
		}
		defer source.Close()
		monitor, err := telemetry.NewMonitor(source, logger, timeout)
		if err != nil {
			return err
		}
		controller, err := mavcommand.NewArmingController(source, mavcommand.DefaultCommandConfig())
		if err != nil {
			return err
		}
		members = append(members, group.FlightMember{Flight: controller, Arming: controller, States: monitor})
		routeMembers = append(routeMembers, group.RouteMember{FlightMember: members[i], Positions: controller})
		monitors = append(monitors, monitor)
	}
	scenarioCtx, cancelScenario := context.WithCancelCause(ctx)
	defer cancelScenario(nil)
	readCtx, cancelReading := context.WithCancel(context.Background())
	defer cancelReading()
	readersDone := make(chan error, len(monitors))
	for _, monitor := range monitors {
		go func() {
			err := monitor.Run(readCtx)
			if err != nil {
				cancelScenario(err)
			}
			readersDone <- err
		}()
	}
	var result error
	if route {
		config := group.DefaultGroupRouteConfig()
		logger.Info("group route check", "vehicles", len(members), "minSeparationM", 4, "config", config)
		result = group.RunGroupRouteCheck(scenarioCtx, routeMembers, logger, config, 4)
	} else {
		config := flight.DefaultFlightCheckConfig()
		config.HoldDuration = 8 * time.Second
		config.MaxDistanceM = 2
		logger.Info("group flight check", "vehicles", len(members), "minSeparationM", 4, "targetAltitudeM", config.TargetAltitudeM)
		result = group.RunGroupFlightCheck(scenarioCtx, members, logger, config, 4)
	}
	cancelReading()
	for range monitors {
		result = errors.Join(result, <-readersDone)
	}
	if result != nil {
		return fmt.Errorf("group flight: %w", result)
	}
	logger.Info("group landed", "vehicles", len(members))
	return nil
}
