// Package sensors хранит показания лидара и сонаров независимо от симулятора и автопилота.
package sensors

import (
	"errors"
	"math"
	"time"
)

// Pose задаёт положение в метрах и ориентацию NED, в радианах, в момент измерения.
type Pose struct{ NorthM, EastM, UpM, RollRad, PitchRad, YawRad float64 }

// Lidar содержит слои кругового сканирования слева направо в системе forward/right/down.
// -1 означает отсутствие отражения до MaxRangeM; NaN и бесконечности запрещены.
type Lidar struct {
	Resolution, Layers                            int
	MinRangeM, MaxRangeM, VerticalFOVRad, HeightM float64
	RangesM                                       []float64
	BearingsRad                                   []float64 `json:",omitempty"`
}

// Sonar задаёт направление FRD, положение датчика и измеренную дальность в метрах.
// BearingRad положителен вправо; ElevationRad положителен вверх.
type Sonar struct {
	BearingRad, ElevationRad, ForwardM, RightM, UpM, RangeM, MaxRangeM float64
}

// Frame — атомарный снимок сенсоров с отдельным поколением контроллера и часами симуляции.
type Frame struct {
	Version    int
	SystemID   uint8
	Session    string
	Sequence   uint64
	TimeS      float64
	Pose       Pose
	Lidar      Lidar
	Sonars     []Sonar
	ReceivedAt time.Time `json:"-"`
}

// Reader возвращает независимую копию последнего кадра; nil означает отсутствие измерений.
type Reader interface{ Snapshot() *Frame }

func finite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

// Validate проверяет размеры, единицы, полноту кругового обзора и числовые значения.
func (f Frame) Validate() error {
	if f.Version != 1 || f.SystemID == 0 || len(f.Session) < 1 || len(f.Session) > 64 || f.Sequence == 0 ||
		!finite(f.TimeS) ||
		f.TimeS < 0 {
		return errors.New("invalid sensor frame identity or time")
	}
	for _, v := range []float64{f.Pose.NorthM, f.Pose.EastM, f.Pose.UpM, f.Pose.RollRad, f.Pose.PitchRad, f.Pose.YawRad} {
		if !finite(v) {
			return errors.New("invalid sensor pose")
		}
	}
	l := f.Lidar
	if l.Resolution < 72 || l.Resolution > 720 || l.Layers < 1 || l.Layers > 8 {
		return errors.New("invalid lidar dimensions")
	}
	if len(l.RangesM) != l.Resolution*l.Layers {
		return errors.New("incomplete lidar image")
	}
	if !finite(l.MinRangeM) || !finite(l.MaxRangeM) || l.MinRangeM <= 0 || l.MaxRangeM <= l.MinRangeM ||
		l.MaxRangeM > 100 {
		return errors.New("invalid lidar range limits")
	}
	if !finite(l.VerticalFOVRad) || l.VerticalFOVRad < 0 || l.VerticalFOVRad > math.Pi/2 {
		return errors.New("invalid lidar field of view")
	}
	if !finite(l.HeightM) || math.Abs(l.HeightM) > 1 {
		return errors.New("invalid lidar mount")
	}
	for _, r := range l.RangesM {
		if !finite(r) || (r != -1 && (r < l.MinRangeM || r > l.MaxRangeM)) {
			return errors.New("invalid lidar range")
		}
	}
	if len(f.Sonars) != 10 {
		return errors.New("expected eight horizontal, upward and downward sonars")
	}
	if len(l.BearingsRad) != 0 {
		if len(l.BearingsRad) != l.Resolution {
			return errors.New("invalid lidar bearing count")
		}
		previous := -math.Pi - 1
		for _, angle := range l.BearingsRad {
			if !finite(angle) || angle < -math.Pi || angle > math.Pi || angle <= previous {
				return errors.New("invalid lidar bearings")
			}
			previous = angle
		}
	}
	for i, s := range f.Sonars {
		for _, v := range []float64{s.BearingRad, s.ElevationRad, s.ForwardM, s.RightM, s.UpM, s.RangeM, s.MaxRangeM} {
			if !finite(v) {
				return errors.New("invalid sonar value")
			}
		}
		if s.RangeM < 0 || s.RangeM > s.MaxRangeM || s.MaxRangeM <= 0 || s.MaxRangeM > 100 ||
			math.Abs(s.ForwardM) > 1 || math.Abs(s.RightM) > 1 || math.Abs(s.UpM) > 1 {
			return errors.New("invalid sonar geometry")
		}
		elevation := 0.0
		if i == 8 {
			elevation = math.Pi / 2
		}
		if i == 9 {
			elevation = -math.Pi / 2
		}
		if math.Abs(s.ElevationRad-elevation) > 1e-6 ||
			(i < 8 && math.Abs(math.Remainder(s.BearingRad-float64(i)*math.Pi/4, 2*math.Pi)) > 1e-6) {
			return errors.New("incomplete sonar directions")
		}
	}
	return nil
}

// Fresh проверяет возраст по местным часам приёма; остановленная симуляция не обновляет кадр.
func (f *Frame) Fresh(now time.Time, maxAge time.Duration) bool {
	return f != nil && maxAge > 0 && !f.ReceivedAt.IsZero() && now.Sub(f.ReceivedAt) >= 0 &&
		now.Sub(f.ReceivedAt) <= maxAge
}

func clone(f *Frame) *Frame {
	if f == nil {
		return nil
	}
	copyFrame := *f
	copyFrame.Lidar.RangesM = append([]float64(nil), f.Lidar.RangesM...)
	copyFrame.Lidar.BearingsRad = append([]float64(nil), f.Lidar.BearingsRad...)
	copyFrame.Sonars = append([]Sonar(nil), f.Sonars...)
	return &copyFrame
}

// Rotate переводит вектор FRD в мировую систему north/east/up с учётом всех углов корпуса.
func (p Pose) Rotate(forward, right, down float64) (north, east, up float64) {
	cr, sr := math.Cos(p.RollRad), math.Sin(p.RollRad)
	cp, sp := math.Cos(p.PitchRad), math.Sin(p.PitchRad)
	cy, sy := math.Cos(p.YawRad), math.Sin(p.YawRad)
	north = cy*cp*forward + (cy*sp*sr-sy*cr)*right + (cy*sp*cr+sy*sr)*down
	east = sy*cp*forward + (sy*sp*sr+cy*cr)*right + (sy*sp*cr-cy*sr)*down
	up = sp*forward - cp*sr*right - cp*cr*down
	return north, east, up
}
