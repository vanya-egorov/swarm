package main

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"net"
	"strconv"
	"time"

	"github.com/vanya-egorov/swarm/internal/mavlink"
	"github.com/vanya-egorov/swarm/internal/vehicle"
)

// runGroupCheck держит все три приёмника активными до завершения посадки группы.
func runGroupCheck(
	ctx context.Context,
	address string,
	systemID int,
	timeout time.Duration,
	logger *slog.Logger,
) error {
	host, portText, err := net.SplitHostPort(address)
	if err != nil {
		return err
	}
	port, err := strconv.Atoi(portText)
	if err != nil || port < 1024 || port > 65515 || systemID > 253 {
		return errors.New("invalid group IDs or ports")
	}
	members := make([]vehicle.FlightMember, 0, 3)
	monitors := make([]*vehicle.Monitor, 0, 3)
	for i := range 3 {
		source, err := mavlink.NewTelemetrySource(
			net.JoinHostPort(host, strconv.Itoa(port+10*i)),
			uint8(systemID+i),
			logger,
		)
		if err != nil {
			return err
		}
		defer source.Close()
		monitor, err := vehicle.NewMonitor(source, logger, timeout)
		if err != nil {
			return err
		}
		controller, err := mavlink.NewArmingController(source, mavlink.DefaultCommandConfig())
		if err != nil {
			return err
		}
		members = append(members, vehicle.FlightMember{Flight: controller, Arming: controller, States: monitor})
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
	config := vehicle.DefaultFlightCheckConfig()
	config.HoldDuration = 8 * time.Second
	config.MaxDistanceM = 2
	logger.Info("group flight check", "vehicles", 3, "minSeparationM", 4, "targetAltitudeM", config.TargetAltitudeM)
	result := vehicle.RunGroupFlightCheck(scenarioCtx, members, logger, config, 4)
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
