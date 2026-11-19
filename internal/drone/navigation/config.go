// Package navigation строит локальную карту по дальномерам и обходит препятствия в своей области.
package navigation

import (
	"errors"
	"math"
	"time"
)

// Config задаёт ограниченное движение в горизонтальной плоскости и размеры безопасного коридора.
type Config struct {
	Enabled                                                                                                          bool
	ResolutionM, ClearanceM, LookaheadM, MaxSpeedMS, BrakeMS2, VerticalBandM, EmergencyM, PoseToleranceM, MaxTiltRad float64
	SensorMaxAge, UpdateInterval                                                                                     time.Duration
	MaxCells                                                                                                         int
}

// DefaultConfig задаёт движение до 0,5 м/с с запасом вокруг корпуса Iris.
func DefaultConfig() Config {
	return Config{Enabled: true, ResolutionM: 0.25, ClearanceM: 0.9, LookaheadM: 0.5, MaxSpeedMS: 0.5, BrakeMS2: 0.5,
		VerticalBandM: 0.5, EmergencyM: 0.5, PoseToleranceM: 0.6, MaxTiltRad: 0.35,
		SensorMaxAge: 300 * time.Millisecond, UpdateInterval: 100 * time.Millisecond, MaxCells: 250000}
}

func finite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

// Validate проверяет разрешение карты, остановочный путь и сроки свежести измерений.
func (c Config) Validate() error {
	for _, v := range []float64{c.ResolutionM, c.ClearanceM, c.LookaheadM, c.MaxSpeedMS, c.BrakeMS2, c.VerticalBandM, c.EmergencyM, c.PoseToleranceM, c.MaxTiltRad} {
		if !finite(v) || v <= 0 {
			return errors.New("invalid navigation geometry or speed")
		}
	}
	if c.ResolutionM < 0.1 || c.ResolutionM > 1 || c.LookaheadM < c.ResolutionM || c.LookaheadM > c.ClearanceM ||
		c.MaxSpeedMS > 1 || c.MaxTiltRad > 0.5 || c.SensorMaxAge < 50*time.Millisecond || c.SensorMaxAge > time.Second ||
		c.UpdateInterval < 20*time.Millisecond || c.UpdateInterval > c.SensorMaxAge/2 || c.MaxCells < 100 || c.MaxCells > 250000 {
		return errors.New("navigation limits outside supported range")
	}
	stop := 0.35 + c.MaxSpeedMS*c.SensorMaxAge.Seconds() + c.MaxSpeedMS*c.MaxSpeedMS/(2*c.BrakeMS2)
	if c.ClearanceM < stop || c.EmergencyM < stop-0.3 {
		return errors.New("navigation clearance is below stopping envelope")
	}
	return nil
}

// Point задаёт север и восток в метрах относительно старта своего аппарата.
type Point struct{ NorthM, EastM float64 }

// Bounds ограничивает разрешённые положения центра дрона его выделенной областью.
type Bounds struct{ Min, Max Point }

func (b Bounds) contains(p Point) bool {
	const epsilon = 1e-6 // Ошибка обратного перевода WGS84 в метры не должна исключать граничную цель.
	return p.NorthM >= b.Min.NorthM-epsilon && p.NorthM <= b.Max.NorthM+epsilon && p.EastM >= b.Min.EastM-epsilon &&
		p.EastM <= b.Max.EastM+epsilon
}

func distance(a, b Point) float64 { return math.Hypot(a.NorthM-b.NorthM, a.EastM-b.EastM) }
