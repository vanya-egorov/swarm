// Package cli parses options and runs the selected drone scenario.
package cli

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"net"
	"os"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
	mavcommand "github.com/vanya-egorov/swarm/internal/mavlink/command"
	mavtelemetry "github.com/vanya-egorov/swarm/internal/mavlink/telemetry"
)

// Run starts the selected CLI scenario.
func Run(ctx context.Context, args []string, logger *slog.Logger) error {
	flags := flag.NewFlagSet("swarm", flag.ContinueOnError)
	flags.SetOutput(os.Stderr)
	listenAddress := flags.String("listen", "127.0.0.1:14560", "UDP-адрес MAVLink")
	systemID := flags.Int("system-id", 1, "ID дрона (1–255)")
	armCheck := flags.Bool("sitl-arm-check", false, "проверка arm/disarm")
	flightCheck := flags.Bool("sitl-flight-check", false, "взлёт и посадка в SITL")
	routeCheck := flags.Bool("sitl-route-check", false, "взлёт, квадрат 2×2 м и посадка в SITL")
	groupCheck := flags.Bool("sitl-group-flight-check", false, "совместный взлёт и посадка трёх SITL")
	groupRoute := flags.Bool(
		"sitl-group-route-check",
		false,
		"совместный квадрат трёх SITL с ожиданием всех участников",
	)
	coverageMission := flags.String("coverage-mission", "", "JSON-задание распределённого покрытия для этого SITL")
	sensorListen := flags.String(
		"sensor-listen",
		"",
		"UDP-приёмник лидара/сонаров; по умолчанию 127.0.0.1:14700+system-id",
	)
	sensorSource := flags.String(
		"sensor-source",
		"",
		"UDP-адрес контроллера сенсоров; по умолчанию 127.0.0.1:14800+system-id",
	)
	coordinationState := flags.String(
		"coordination-state-dir",
		"projectData/run/coordination",
		"каталог голосов и маркеров задания",
	)
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

	checks := 0
	for _, enabled := range []bool{*armCheck, *flightCheck, *routeCheck, *groupCheck, *groupRoute, *coverageMission != ""} {
		if enabled {
			checks++
		}
	}
	if checks > 1 {
		return fmt.Errorf("choose one SITL check")
	}
	if checks > 0 {
		host, _, err := net.SplitHostPort(*listenAddress)
		if err != nil || !net.ParseIP(host).IsLoopback() {
			return fmt.Errorf("SITL check needs loopback address: %q", *listenAddress)
		}
	}

	if *groupCheck || *groupRoute {
		return runGroupCheck(ctx, *listenAddress, *systemID, *timeout, logger, *groupRoute)
	}
	if *coverageMission != "" {
		return runCoverageCheck(
			ctx,
			*coverageMission,
			*coordinationState,
			*listenAddress,
			uint8(*systemID),
			*timeout,
			logger,
			*sensorListen, *sensorSource,
		)
	}

	source, err := mavtelemetry.NewTelemetrySource(*listenAddress, uint8(*systemID), logger)
	if err != nil {
		return fmt.Errorf("MAVLink source: %w", err)
	}
	defer source.Close()

	monitor, err := telemetry.NewMonitor(source, logger, *timeout)
	if err != nil {
		return fmt.Errorf("monitor: %w", err)
	}
	logger.Info("waiting for heartbeat", "listen", *listenAddress, "systemId", *systemID, "timeout", timeout.String())

	if checks > 0 {
		controller, err := mavcommand.NewArmingController(source, mavcommand.DefaultCommandConfig())
		if err != nil {
			return fmt.Errorf("controller: %w", err)
		}
		if *routeCheck {
			return runScenario(ctx, monitor, func(scenarioCtx context.Context) error {
				return route.RunRouteCheck(
					scenarioCtx,
					controller,
					controller,
					controller,
					monitor,
					logger,
					route.DefaultRouteCheckConfig(),
				)
			})
		}
		if *flightCheck {
			return runFlightCheck(ctx, monitor, controller, logger)
		}
		return runArmCheck(ctx, monitor, controller, logger)
	}

	err = monitor.Run(ctx)
	if errors.Is(err, telemetry.ErrHeartbeatTimeout) {
		return fmt.Errorf("vehicle %d offline: %w", *systemID, err)
	}
	return err
}

// runFlightCheck оставляет чтение телеметрии активным до подтверждения посадки.
func runFlightCheck(
	ctx context.Context,
	monitor *telemetry.Monitor,
	controller *mavcommand.ArmingController,
	logger *slog.Logger,
) error {
	return runScenario(ctx, monitor, func(scenarioCtx context.Context) error {
		return flight.RunFlightCheck(
			scenarioCtx,
			controller,
			controller,
			monitor,
			logger,
			flight.DefaultFlightCheckConfig(),
		)
	})
}

// runArmCheck оставляет чтение телеметрии активным до завершения disarm.
func runArmCheck(
	ctx context.Context,
	monitor *telemetry.Monitor,
	controller flight.ArmingController,
	logger *slog.Logger,
) error {
	return runScenario(ctx, monitor, func(scenarioCtx context.Context) error {
		return flight.RunArmCheck(scenarioCtx, controller, monitor, logger, flight.DefaultArmCheckConfig())
	})
}

// runScenario сохраняет приём телеметрии до завершения очистки сценария.
func runScenario(ctx context.Context, monitor *telemetry.Monitor, scenario func(context.Context) error) error {
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
