package vehicle

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"
)

// landAndConfirm включает LAND при необходимости и ждёт безопасного разоружения на земле.
func landAndConfirm(
	ctx context.Context,
	flight FlightController,
	arming ArmingController,
	states StateReader,
	before State,
	config FlightCheckConfig,
	logger *slog.Logger,
) error {
	state := states.Snapshot()
	if freshState(state, config.TelemetryMaxAge) && sameVehicle(before, state) && !state.Armed &&
		landedOnGround(state.Position) {
		return nil
	}
	landErr := flightCheckCommand(ctx, config.CommandTimeout, flight.Land)
	if landErr != nil {
		logger.Warn("LAND unconfirmed", "systemId", before.SystemID, "error", landErr)
	}
	landingCtx, cancel := context.WithTimeout(ctx, config.LandingTimeout)
	defer cancel()
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()
	var groundSince time.Time
	manualDisarmAttempted := false
	var manualDisarmErr error
	for {
		if err := landingCtx.Err(); err != nil {
			return errors.Join(landErr, manualDisarmErr, fmt.Errorf("disarm unconfirmed: %w", err))
		}
		state = states.Snapshot()
		if freshState(state, config.TelemetryMaxAge) && sameVehicle(before, state) {
			if !state.Armed && landedOnGround(state.Position) {
				logger.Info("landed", "systemId", before.SystemID, "altitudeM", state.Position.RelativeAltitudeM)
				return nil
			}
			if state.Armed && landedOnGround(state.Position) {
				if groundSince.IsZero() {
					groundSince = time.Now()
				}
				if !manualDisarmAttempted && time.Since(groundSince) >= 2*time.Second {
					manualDisarmAttempted = true
					manualDisarmErr = flightCheckCommand(
						landingCtx,
						config.CommandTimeout,
						func(commandCtx context.Context) error { return arming.SetArmed(commandCtx, false) },
					)
					if manualDisarmErr != nil {
						logger.Warn("disarm unconfirmed", "systemId", before.SystemID, "error", manualDisarmErr)
					}
				}
			} else {
				groundSince = time.Time{}
			}
		}
		select {
		case <-landingCtx.Done():
		case <-ticker.C:
		}
	}
}

// landedOnGround подтверждает почти нулевую высоту и малые скорости перед disarm.
func landedOnGround(position *Position) bool {
	return grounded(position, 0.15)
}
