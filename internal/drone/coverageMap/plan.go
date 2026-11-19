package coverageMap

import (
	"errors"
	"fmt"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/route"
)

// Assignment содержит выделенную дрону область и последовательность целей в координатах карты.
type Assignment struct {
	Region
	Waypoints []route.Waypoint
}

// Plan назначает каждому ID полосу и маршрут змейкой; при ошибке возвращает только nil.
// Внутренности областей и маршруты разных аппаратов не пересекаются.
// Подлёт из места старта, возврат и физическая дистанция между дронами требуют отдельного планирования.
func Plan(area Area, systemIDs []uint8, config Config) ([]Assignment, error) {
	if err := config.Validate(); err != nil {
		return nil, err
	}
	regions, err := Decompose(area, systemIDs, config.Direction)
	if err != nil {
		return nil, err
	}
	assignments := make([]Assignment, len(regions))
	total := 0
	for i, region := range regions {
		points, err := GenerateRoute(region.Area, config)
		if err != nil {
			return nil, fmt.Errorf("vehicle %d coverage: %w", region.SystemID, err)
		}
		total += len(points)
		if total > maxWaypoints {
			return nil, errors.New("coverage plan: too many waypoints")
		}
		assignments[i] = Assignment{Region: region, Waypoints: points}
	}
	return assignments, nil
}

// WaypointsFrom переводит цели карты в смещения от места старта конкретного дрона.
// origin — положение старта в общей системе карты; высота остаётся высотой над home.
// Результат можно передать в route.TargetsFrom или RouteCheckConfig после проверки полётных ограничений.
func (a Assignment) WaypointsFrom(origin Point) ([]route.Waypoint, error) {
	if !coordinates.Finite(origin.NorthM) || !coordinates.Finite(origin.EastM) {
		return nil, errors.New("coverage origin: coordinates must be finite")
	}
	points := make([]route.Waypoint, len(a.Waypoints))
	for i, p := range a.Waypoints {
		p.NorthM -= origin.NorthM
		p.EastM -= origin.EastM
		if !coordinates.Finite(p.NorthM) || !coordinates.Finite(p.EastM) ||
			!coordinates.Finite(p.AltitudeM) || p.AltitudeM <= 0 {
			return nil, fmt.Errorf("coverage waypoint %d: invalid offset or altitude", i+1)
		}
		points[i] = p
	}
	return points, nil
}
