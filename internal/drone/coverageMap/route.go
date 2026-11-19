package coverageMap

import (
	"errors"
	"math"

	"github.com/vanya-egorov/swarm/internal/drone/route"
)

// Ограничение защищает от выделения памяти при слишком малом шаге проходов.
const maxWaypoints = 100000

// GenerateRoute строит змейку внутри области, включая покрытие краёв и углов.
// Точки заданы относительно общего начала карты; отрезки между ними прямые.
// Центры проходов равномерно распределены с шагом не больше LaneSpacingM.
func GenerateRoute(area Area, config Config) ([]route.Waypoint, error) {
	if err := area.Validate(); err != nil {
		return nil, err
	}
	if err := config.Validate(); err != nil {
		return nil, err
	}
	minAlong, maxAlong := area.Min.NorthM, area.Max.NorthM
	minCross, maxCross := area.Min.EastM, area.Max.EastM
	if config.Direction == EastWest {
		minAlong, maxAlong = minCross, maxCross
		minCross, maxCross = area.Min.NorthM, area.Max.NorthM
	}
	count := math.Ceil((maxCross - minCross) / config.LaneSpacingM)
	if config.BoundaryMarginM > 0 {
		if config.BoundaryMarginM > (maxCross-minCross)/2 || config.BoundaryMarginM > (maxAlong-minAlong)/2 {
			return nil, errors.New("coverage route: region too narrow for boundary margin")
		}
		count = math.Ceil((maxCross-minCross-2*config.BoundaryMarginM)/config.LaneSpacingM) + 1
	}
	if math.IsInf(count, 0) || count > maxWaypoints/2 {
		return nil, errors.New("coverage route: too many waypoints")
	}
	lanes := max(1, int(count))
	margin := math.Min(config.SwathWidthM/2, (maxAlong-minAlong)/2)
	start, end := minAlong+margin, maxAlong-margin
	if maxAlong-minAlong <= config.SwathWidthM {
		end = start
	}
	points := make([]route.Waypoint, 0, 2*lanes)
	previousCross := minCross
	for i := range lanes {
		cross := minCross + (maxCross-minCross)*((float64(i)+0.5)/float64(lanes))
		if config.BoundaryMarginM > 0 {
			cross = minCross + (maxCross-minCross)/2
			if lanes > 1 {
				cross = minCross + config.BoundaryMarginM +
					(maxCross-minCross-2*config.BoundaryMarginM)*float64(i)/float64(lanes-1)
				if i == lanes-1 {
					cross = maxCross - config.BoundaryMarginM
				}
			}
		}
		if cross <= previousCross || cross >= maxCross || start <= minAlong || start >= maxAlong ||
			end <= minAlong || end >= maxAlong {
			return nil, errors.New("coverage route: coordinates too large for requested resolution")
		}
		previousCross = cross
		from, to := start, end
		if i%2 != 0 {
			from, to = to, from
		}
		points = append(points, waypoint(from, cross, config))
		if from != to {
			points = append(points, waypoint(to, cross, config))
		}
	}
	return points, nil
}

func waypoint(along, cross float64, config Config) route.Waypoint {
	if config.Direction == EastWest {
		return route.Waypoint{NorthM: cross, EastM: along, AltitudeM: config.AltitudeM}
	}
	return route.Waypoint{NorthM: along, EastM: cross, AltitudeM: config.AltitudeM}
}
