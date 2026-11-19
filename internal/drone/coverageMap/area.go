// Package coverageMap делит прямоугольную область между дронами и строит маршруты покрытия.
package coverageMap

import (
	"errors"
	"fmt"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
)

// Point задаёт смещения в метрах от общего начала координат карты.
type Point struct{ NorthM, EastM float64 }

// Area задаёт прямоугольник, стороны которого направлены на север и восток.
type Area struct{ Min, Max Point }

// Validate проверяет конечность координат и положительные размеры области.
func (a Area) Validate() error {
	if !coordinates.Finite(a.Min.NorthM) || !coordinates.Finite(a.Min.EastM) ||
		!coordinates.Finite(a.Max.NorthM) || !coordinates.Finite(a.Max.EastM) {
		return errors.New("coverage area: coordinates must be finite")
	}
	north, east := a.Max.NorthM-a.Min.NorthM, a.Max.EastM-a.Min.EastM
	if !coordinates.Finite(north) || !coordinates.Finite(east) || north <= 0 || east <= 0 {
		return errors.New("coverage area: dimensions must be finite and positive")
	}
	return nil
}

// Direction задаёт направление длинных проходов змейки.
type Direction uint8

const (
	// NorthSouth направляет проходы с юга на север и обратно, разделяя область по восточной оси.
	NorthSouth Direction = iota
	// EastWest направляет проходы с запада на восток и обратно, разделяя область по северной оси.
	EastWest
)

func (d Direction) validate() error {
	if d != NorthSouth && d != EastWest {
		return errors.New("coverage direction: expected north/south or east/west")
	}
	return nil
}

// Region закрепляет подобласть за конкретным MAVLink system ID.
type Region struct {
	SystemID uint8
	Area     Area
}

// Decompose делит область на равные полосы поперёк проходов змейки.
// Порядок systemIDs определяет назначение полос от Min к Max; общие границы допустимы.
func Decompose(area Area, systemIDs []uint8, direction Direction) ([]Region, error) {
	if err := area.Validate(); err != nil {
		return nil, err
	}
	if err := direction.validate(); err != nil {
		return nil, err
	}
	if len(systemIDs) == 0 || len(systemIDs) > 255 {
		return nil, errors.New("coverage group: expected 1 to 255 drones")
	}
	seen := [256]bool{}
	for _, id := range systemIDs {
		if id == 0 || seen[id] {
			return nil, fmt.Errorf("coverage group: invalid or duplicate system ID %d", id)
		}
		seen[id] = true
	}
	minCross, maxCross := area.Min.EastM, area.Max.EastM
	if direction == EastWest {
		minCross, maxCross = area.Min.NorthM, area.Max.NorthM
	}
	regions := make([]Region, len(systemIDs))
	boundary := minCross
	for i, id := range systemIDs {
		next := minCross + (maxCross-minCross)*(float64(i+1)/float64(len(systemIDs)))
		if i == len(systemIDs)-1 {
			next = maxCross
		}
		subarea := area
		if direction == NorthSouth {
			subarea.Min.EastM, subarea.Max.EastM = boundary, next
		} else {
			subarea.Min.NorthM, subarea.Max.NorthM = boundary, next
		}
		if err := subarea.Validate(); err != nil {
			return nil, fmt.Errorf("vehicle %d region: %w", id, err)
		}
		regions[i] = Region{SystemID: id, Area: subarea}
		boundary = next
	}
	return regions, nil
}
