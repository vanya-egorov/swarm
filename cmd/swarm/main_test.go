package main

import (
	"context"
	"log/slog"
	"strings"
	"testing"
)

func TestSITLScenarioFlags(t *testing.T) {
	for _, args := range [][]string{
		{"-sitl-route-check", "-sitl-flight-check"},
		{"-sitl-group-route-check", "-sitl-group-flight-check"},
		{"-sitl-group-route-check", "-listen", "0.0.0.0:14560"},
		{"-sitl-group-flight-check", "-sitl-route-check"},
		{"-sitl-group-flight-check", "-listen", "0.0.0.0:14560"},
		{"-sitl-route-check", "-sitl-arm-check"},
		{"-sitl-route-check", "-listen", "0.0.0.0:14560"},
	} {
		err := run(context.Background(), args, slog.New(slog.DiscardHandler))
		if err == nil || !strings.Contains(err.Error(), "choose one") && !strings.Contains(err.Error(), "loopback") {
			t.Fatalf("flags %v: %v", args, err)
		}
	}
}
