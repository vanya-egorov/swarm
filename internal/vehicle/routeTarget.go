package vehicle

import (
	"context"
	"math"
	"time"
)

func followTarget(
	ctx context.Context,
	positions PositionController,
	states StateReader,
	before State,
	config RouteCheckConfig,
	target PositionTarget,
) error {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	var firstSent, lastSent, arrived time.Time
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		state := states.Snapshot()
		if err := flightCheckState(state, before, config.Flight, true); err != nil {
			return err
		}
		if lastSent.IsZero() || time.Since(lastSent) >= time.Second {
			sent := time.Now()
			if err := flightCheckCommand(ctx, config.Flight.CommandTimeout, func(c context.Context) error { return positions.SetPositionTarget(c, target) }); err != nil {
				return err
			}
			lastSent = sent
			if firstSent.IsZero() {
				firstSent = sent
			}
		}
		p := state.Position
		atTarget := p.ReceivedAt.After(firstSent) && nearTarget(p, target, config)
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

func nearTarget(p *Position, target PositionTarget, config RouteCheckConfig) bool {
	return horizontalDistance(
		p.LatitudeDeg,
		p.LongitudeDeg,
		target.LatitudeDeg,
		target.LongitudeDeg,
	) <= config.HorizontalToleranceM &&
		math.Abs(p.RelativeAltitudeM-target.RelativeAltitudeM) <= config.AltitudeToleranceM &&
		math.Hypot(p.NorthVelocityMS, p.EastVelocityMS) < 0.3 &&
		math.Abs(p.DownVelocityMS) < 0.3
}
