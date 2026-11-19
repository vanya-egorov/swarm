package cli

import (
	"context"
	"log/slog"
	"os"
	"path/filepath"
	"testing"

	"github.com/vanya-egorov/swarm/internal/drone/coverageMap"
)

func TestCoverageMissionExample(t *testing.T) {
	config, err := readCoverageMission("../../configs/coverage-example.json")
	if err != nil {
		t.Fatal(err)
	}
	for id := uint8(1); id <= 3; id++ {
		if _, err := coverageMap.PrepareMission(config, id); err != nil {
			t.Fatalf("example for ID %d: %v", id, err)
		}
	}
}

func TestCoverageMissionRejectsUnknownAndTrailingJSON(t *testing.T) {
	for _, data := range []string{`{"Unknown":true}`, `{} {}`, `{`} {
		path := filepath.Join(t.TempDir(), "mission.json")
		if err := os.WriteFile(path, []byte(data), 0600); err != nil {
			t.Fatal(err)
		}
		if _, err := readCoverageMission(path); err == nil {
			t.Fatal("malformed mission accepted")
		}
	}
}

func TestCoverageFlagsRejectConflictingModes(t *testing.T) {
	for _, args := range [][]string{
		{"-coverage-mission", "missing.json", "-sitl-route-check"},
		{"-coverage-mission", "missing.json", "-listen", "0.0.0.0:14560"},
		{"-coverage-mission", "missing.json", "-system-id", "256"},
	} {
		if err := Run(context.Background(), args, slog.New(slog.DiscardHandler)); err == nil {
			t.Fatal("invalid coverage flags accepted")
		}
	}
}
