package coverageMap

import (
	"errors"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
)

// Config задаёт геометрию покрытия и высоту точек над home каждого аппарата.
// SwathWidthM — сторона гарантированного квадратного пятна обзора на земле.
// LaneSpacingM — максимальный шаг между проходами, не больше ширины обзора.
// BoundaryMarginM — минимальный отступ траектории от границ; не больше половины ширины обзора.
type Config struct {
	SwathWidthM, LaneSpacingM, AltitudeM float64
	Direction                            Direction
	BoundaryMarginM                      float64
}

// DefaultConfig задаёт северные/южные проходы с шириной и шагом 1 м на высоте 1,5 м.
func DefaultConfig() Config {
	return Config{SwathWidthM: 1, LaneSpacingM: 1, AltitudeM: 1.5, Direction: NorthSouth}
}

// Validate проверяет, что параметры позволяют покрыть область без пропусков.
func (c Config) Validate() error {
	if !coordinates.Finite(c.SwathWidthM) || c.SwathWidthM <= 0 ||
		!coordinates.Finite(c.LaneSpacingM) || c.LaneSpacingM <= 0 || c.LaneSpacingM > c.SwathWidthM {
		return errors.New("coverage config: expected 0 < lane spacing <= swath width")
	}
	if !coordinates.Finite(c.AltitudeM) || c.AltitudeM <= 0 {
		return errors.New("coverage config: altitude must be finite and positive")
	}
	if !coordinates.Finite(c.BoundaryMarginM) || c.BoundaryMarginM < 0 || c.BoundaryMarginM > c.SwathWidthM/2 {
		return errors.New("coverage config: boundary margin must be between zero and half swath width")
	}
	return c.Direction.validate()
}
