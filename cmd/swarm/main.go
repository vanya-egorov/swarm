// Command swarm is the single executable entry point.
package main

import (
	"context"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/vanya-egorov/swarm/internal/cli"
)

func main() {
	logger := slog.New(slog.NewJSONHandler(os.Stderr, nil))
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	err := cli.Run(ctx, os.Args[1:], logger)
	stop()
	if err != nil {
		logger.Error("stopped", "error", err)
		os.Exit(1)
	}
	logger.Info("stopped")
}
