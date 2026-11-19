package cli

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log/slog"
	"os"
	"path/filepath"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordination"
	"github.com/vanya-egorov/swarm/internal/drone/coverageMap"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
	mavcommand "github.com/vanya-egorov/swarm/internal/mavlink/command"
	mavtelemetry "github.com/vanya-egorov/swarm/internal/mavlink/telemetry"
)

func readCoverageMission(path string) (coverageMap.MissionConfig, error) {
	f, err := os.Open(path)
	if err != nil {
		return coverageMap.MissionConfig{}, err
	}
	defer func() { _ = f.Close() }()
	config := coverageMap.DefaultMissionConfig()
	d := json.NewDecoder(io.LimitReader(f, 1<<20))
	d.DisallowUnknownFields()
	if err := d.Decode(&config); err != nil {
		return coverageMap.MissionConfig{}, err
	}
	if err := d.Decode(new(any)); !errors.Is(err, io.EOF) {
		return coverageMap.MissionConfig{}, errors.New("unexpected mission JSON suffix")
	}
	return config, nil
}

func runCoverageCheck(ctx context.Context, path, stateDir, address string, systemID uint8,
	timeout time.Duration, logger *slog.Logger) error {
	config, err := readCoverageMission(path)
	if err != nil {
		return fmt.Errorf("coverage mission: %w", err)
	}
	prepared, err := coverageMap.PrepareMission(config, systemID)
	if err != nil {
		return fmt.Errorf("coverage preflight: %w", err)
	}
	transport, err := coordination.NewUDPTransport(prepared.Config.Network)
	if err != nil {
		return fmt.Errorf("coverage UDP: %w", err)
	}
	defer func() { _ = transport.Close() }()
	source, err := mavtelemetry.NewTelemetrySource(address, systemID, logger)
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
	statePath := filepath.Join(stateDir, fmt.Sprintf("%s-%d.json", prepared.Config.Network.ClusterID, systemID))
	runner := coverageMap.MissionRunner{Config: config, SystemID: systemID, Transport: transport,
		Store: coordination.FileStore{Path: statePath}, StartMarker: statePath + ".flight",
		Vehicle: coverageMap.Vehicle{
			Flight:    controller,
			Arming:    controller,
			Positions: controller,
			States:    monitor,
		}, Logger: logger}
	logger.Info("coverage agent started", "systemId", systemID, "clusterId", prepared.Config.Network.ClusterID,
		"waypoints", len(prepared.Route.Waypoints), "peers", len(config.Network.Peers))
	return runScenario(ctx, monitor, runner.Run)
}
