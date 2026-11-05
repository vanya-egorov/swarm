package vehicle

import (
	"errors"
	"fmt"
	"math"
	"time"
)

// RouteCheckConfig задаёт маршрут, допуски прибытия и тайм-ауты.
type RouteCheckConfig struct {
	Flight                                   FlightCheckConfig
	Waypoints                                []Waypoint
	WaypointTimeout, ArrivalDuration         time.Duration
	HorizontalToleranceM, AltitudeToleranceM float64
}

// DefaultRouteCheckConfig задаёт квадрат 2×2 м с возвращением к месту старта.
func DefaultRouteCheckConfig() RouteCheckConfig {
	return RouteCheckConfig{Flight: DefaultFlightCheckConfig(),
		Waypoints:       []Waypoint{{2, 0, 1.5}, {2, 2, 1.5}, {0, 2, 1.5}, {0, 0, 1.5}},
		WaypointTimeout: 25 * time.Second, ArrivalDuration: time.Second,
		HorizontalToleranceM: 0.4, AltitudeToleranceM: 0.25}
}

func (c RouteCheckConfig) validate() error {
	if err := c.Flight.validate(); err != nil {
		return err
	}
	if len(c.Waypoints) == 0 || len(c.Waypoints) > 20 || c.WaypointTimeout <= 0 || c.ArrivalDuration <= 0 ||
		c.ArrivalDuration >= c.WaypointTimeout {
		return errors.New("invalid route length or timing")
	}
	if !finite(c.HorizontalToleranceM) || c.HorizontalToleranceM <= 0 || !finite(c.AltitudeToleranceM) ||
		c.AltitudeToleranceM <= 0 {
		return errors.New("invalid arrival tolerance")
	}
	for i, p := range c.Waypoints {
		if !finite(p.NorthM) || !finite(p.EastM) || !finite(p.AltitudeM) || p.AltitudeM < 0.8 || p.AltitudeM > 2 ||
			p.AltitudeM+c.AltitudeToleranceM >= c.Flight.MaxAltitudeM ||
			math.Hypot(p.NorthM, p.EastM)+c.HorizontalToleranceM >= c.Flight.MaxDistanceM {
			return fmt.Errorf("waypoint %d outside flight limits", i+1)
		}
	}
	return nil
}
