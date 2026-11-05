package vehicle

import (
	"context"
	"errors"
	"math"
)

const metersPerDegree = 111195.0

// Waypoint задаёт смещение от места старта и высоту относительно home автопилота.
type Waypoint struct{ NorthM, EastM, AltitudeM float64 }

// PositionTarget использует WGS84 и высоту относительно home, а не MSL.
type PositionTarget struct{ LatitudeDeg, LongitudeDeg, RelativeAltitudeM float64 }

// Validate проверяет координаты и допустимую высоту цели SITL.
func (p PositionTarget) Validate() error {
	if !finite(p.LatitudeDeg) || math.Abs(p.LatitudeDeg) > 90 || !finite(p.LongitudeDeg) ||
		math.Abs(p.LongitudeDeg) > 180 ||
		!finite(p.RelativeAltitudeM) ||
		p.RelativeAltitudeM < 0.8 ||
		p.RelativeAltitudeM > 2 {
		return errors.New("invalid position target")
	}
	return nil
}

// PositionController отправляет цель без ACK: достижение проверяет сценарий по телеметрии.
type PositionController interface {
	SetPositionTarget(context.Context, PositionTarget) error
}

func validPosition(p *Position) bool {
	return p != nil && finite(p.LatitudeDeg) && math.Abs(p.LatitudeDeg) <= 90 && finite(p.LongitudeDeg) &&
		math.Abs(p.LongitudeDeg) <= 180 &&
		finite(p.RelativeAltitudeM) &&
		finite(p.NorthVelocityMS) &&
		finite(p.EastVelocityMS) &&
		finite(p.DownVelocityMS)
}

func routeTargets(origin *Position, waypoints []Waypoint) ([]PositionTarget, error) {
	if !validPosition(origin) || math.Abs(origin.LatitudeDeg) > 85 {
		return nil, errors.New("invalid route origin (latitude must be within ±85 degrees)")
	}
	targets := make([]PositionTarget, len(waypoints))
	for i, p := range waypoints {
		lon := origin.LongitudeDeg + p.EastM/(metersPerDegree*math.Cos(origin.LatitudeDeg*math.Pi/180))
		lon = math.Mod(lon+540, 360) - 180
		targets[i] = PositionTarget{origin.LatitudeDeg + p.NorthM/metersPerDegree, lon, p.AltitudeM}
		if err := targets[i].Validate(); err != nil {
			return nil, err
		}
	}
	return targets, nil
}

func horizontalDistance(lat1, lon1, lat2, lon2 float64) float64 {
	dlon := math.Remainder(lon2-lon1, 360)
	return math.Hypot((lat2-lat1)*metersPerDegree, dlon*metersPerDegree*math.Cos(lat1*math.Pi/180))
}
