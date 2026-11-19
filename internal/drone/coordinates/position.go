// Package geo contains coordinate checks and distance calculations for drone positions.
package coordinates

import (
	"math"

	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

const MetersPerDegree = 111195.0

// ValidPosition reports whether a position contains finite, in-range coordinates and velocities.
func ValidPosition(p *telemetry.Position) bool {
	return p != nil && Finite(p.LatitudeDeg) && math.Abs(p.LatitudeDeg) <= 90 &&
		Finite(p.LongitudeDeg) && math.Abs(p.LongitudeDeg) <= 180 &&
		Finite(p.RelativeAltitudeM) && Finite(p.NorthVelocityMS) &&
		Finite(p.EastVelocityMS) && Finite(p.DownVelocityMS)
}

// HorizontalDistance estimates the horizontal distance in meters between two WGS84 points.
func HorizontalDistance(lat1, lon1, lat2, lon2 float64) float64 {
	dlon := math.Remainder(lon2-lon1, 360)
	return math.Hypot((lat2-lat1)*MetersPerDegree, dlon*MetersPerDegree*math.Cos(lat1*math.Pi/180))
}

// Finite reports whether value is neither NaN nor infinite.
func Finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
