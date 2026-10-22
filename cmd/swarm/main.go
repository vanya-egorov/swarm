package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"net"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/vanya-egorov/swarm/internal/mavlink"
	"github.com/vanya-egorov/swarm/internal/vehicle"
)

func main() {
	logger := slog.New(slog.NewJSONHandler(os.Stderr, nil))
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	if err := run(ctx, os.Args[1:], logger); err != nil {
		logger.Error("stopped", "error", err)
		os.Exit(1)
	}
	logger.Info("stopped")
}

func run(ctx context.Context, args []string, logger *slog.Logger) error {
	flags := flag.NewFlagSet("swarm", flag.ContinueOnError)
	flags.SetOutput(os.Stderr)
	listenAddress := flags.String("listen", "127.0.0.1:14560", "UDP-адрес MAVLink")
	systemID := flags.Int("system-id", 1, "ID дрона (1–255)")
	armCheck := flags.Bool("sitl-arm-check", false, "проверка arm/disarm")
	flightCheck := flags.Bool("sitl-flight-check", false, "взлёт и посадка в SITL")
	timeout := flags.Duration("heartbeat-timeout", 10*time.Second, "тайм-аут heartbeat")
	if err := flags.Parse(args); err != nil {
		return fmt.Errorf("flags: %w", err)
	}
	if flags.NArg() != 0 {
		return fmt.Errorf("unexpected arguments: %v", flags.Args())
	}
	if *systemID < 1 || *systemID > 255 {
		return fmt.Errorf("system ID out of range: %d", *systemID)
	}

	if *armCheck && *flightCheck {
		return fmt.Errorf("choose one SITL check")
	}
	if *armCheck || *flightCheck {
		host, _, err := net.SplitHostPort(*listenAddress)
		if err != nil || !net.ParseIP(host).IsLoopback() {
			return fmt.Errorf("SITL check needs loopback address: %q", *listenAddress)
		}
	}

	source, err := mavlink.NewTelemetrySource(*listenAddress, uint8(*systemID), logger)
	if err != nil {
		return fmt.Errorf("MAVLink source: %w", err)
	}
	defer source.Close()

	monitor, err := vehicle.NewMonitor(source, logger, *timeout)
	if err != nil {
		return fmt.Errorf("monitor: %w", err)
	}
	logger.Info("waiting for heartbeat", "listen", *listenAddress, "systemId", *systemID, "timeout", timeout.String())

	if *armCheck || *flightCheck {
		controller, err := mavlink.NewArmingController(source, mavlink.DefaultCommandConfig())
		if err != nil {
			return fmt.Errorf("controller: %w", err)
		}
		if *flightCheck {
			return runFlightCheck(ctx, monitor, controller, logger)
		}
		return runArmCheck(ctx, monitor, controller, logger)
	}

	err = monitor.Run(ctx)
	if errors.Is(err, vehicle.ErrHeartbeatTimeout) {
		return fmt.Errorf("vehicle %d offline: %w", *systemID, err)
	}
	return err
}

// runFlightCheck оставляет чтение телеметрии активным до подтверждения посадки.
func runFlightCheck(ctx context.Context, monitor *vehicle.Monitor, controller *mavlink.ArmingController, logger *slog.Logger) error {
	return runScenario(ctx, monitor, func(scenarioCtx context.Context) error {
		return vehicle.RunFlightCheck(scenarioCtx, controller, controller, monitor, logger, vehicle.DefaultFlightCheckConfig())
	})
}

// runArmCheck оставляет чтение телеметрии активным до завершения disarm.
func runArmCheck(ctx context.Context, monitor *vehicle.Monitor, controller vehicle.ArmingController, logger *slog.Logger) error {
	return runScenario(ctx, monitor, func(scenarioCtx context.Context) error {
		return vehicle.RunArmCheck(scenarioCtx, controller, monitor, logger, vehicle.DefaultArmCheckConfig())
	})
}

// runScenario сохраняет приём телеметрии до завершения очистки сценария.
func runScenario(ctx context.Context, monitor *vehicle.Monitor, scenario func(context.Context) error) error {
	readCtx, stopReading := context.WithCancel(context.Background())
	defer stopReading()
	scenarioCtx, stopScenario := context.WithCancelCause(ctx)
	defer stopScenario(nil)
	readerDone := make(chan error, 1)
	go func() {
		err := monitor.Run(readCtx)
		if err != nil {
			stopScenario(err)
		}
		readerDone <- err
	}()
	err := scenario(scenarioCtx)
	stopReading()
	return errors.Join(err, <-readerDone)
}
