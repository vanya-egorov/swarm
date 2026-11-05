package vehicle

import (
	"errors"
	"fmt"
	"math"
	"time"
)

// FlightCheckConfig ограничивает высоту, ожидания и длительность пробного полёта SITL.
type FlightCheckConfig struct {
	ReadyTimeout    time.Duration
	StableDuration  time.Duration
	CommandTimeout  time.Duration
	ClimbTimeout    time.Duration
	HoldDuration    time.Duration
	LandingTimeout  time.Duration
	TelemetryMaxAge time.Duration
	TargetAltitudeM float64
	MaxAltitudeM    float64
	MaxDistanceM    float64
}

// DefaultFlightCheckConfig задаёт короткий полёт одного Iris на 1,5 метра.
func DefaultFlightCheckConfig() FlightCheckConfig {
	return FlightCheckConfig{
		ReadyTimeout: 30 * time.Second, StableDuration: 20 * time.Second,
		CommandTimeout: 10 * time.Second, ClimbTimeout: 25 * time.Second,
		HoldDuration: 2 * time.Second, LandingTimeout: 60 * time.Second,
		TelemetryMaxAge: 5 * time.Second, TargetAltitudeM: 1.5,
		MaxAltitudeM: 2.5, MaxDistanceM: 5,
	}
}

// validate проверяет время и пространственные границы полёта.
func (c FlightCheckConfig) validate() error {
	if c.ReadyTimeout <= 0 || c.StableDuration < 0 || c.CommandTimeout <= 0 || c.ClimbTimeout <= 0 ||
		c.HoldDuration < 0 || c.LandingTimeout <= 0 || c.TelemetryMaxAge <= 0 {
		return errors.New("invalid flight timing")
	}
	if !finite(c.TargetAltitudeM) || c.TargetAltitudeM < 0.8 || c.TargetAltitudeM > 2 {
		return fmt.Errorf("invalid target altitude: %g m", c.TargetAltitudeM)
	}
	if !finite(c.MaxAltitudeM) || c.MaxAltitudeM <= c.TargetAltitudeM {
		return fmt.Errorf("invalid altitude limit: %g m", c.MaxAltitudeM)
	}
	if !finite(c.MaxDistanceM) || c.MaxDistanceM <= 0 {
		return fmt.Errorf("invalid distance limit: %g m", c.MaxDistanceM)
	}
	return nil
}

// finite проверяет, что измеримый предел не равен NaN или бесконечности.
func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
