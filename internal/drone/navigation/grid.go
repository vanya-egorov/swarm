package navigation

import (
	"errors"
	"math"

	"github.com/vanya-egorov/swarm/internal/drone/sensors"
)

const (
	unknown uint8 = iota
	free
	occupied
)

// Grid хранит наблюдённую свободную область; неизвестное пространство запрещено для движения.
type Grid struct {
	config         Config
	bounds         Bounds
	origin         Point
	width, height  int
	cells          []uint8
	blocked        []bool
	sequence       uint64
	session        string
	frontierVisits map[int]int
	lastFrontier   int
	exceptionStart int
}

// NewGrid выделяет ограниченную сетку с запасом для проверки габаритов у границ области.
func NewGrid(bounds Bounds, config Config) (*Grid, error) {
	if err := config.Validate(); err != nil {
		return nil, err
	}
	for _, v := range []float64{bounds.Min.NorthM, bounds.Min.EastM, bounds.Max.NorthM, bounds.Max.EastM} {
		if !finite(v) {
			return nil, errors.New("invalid navigation bounds")
		}
	}
	pad := (math.Ceil((config.ClearanceM+math.Sqrt2*config.ResolutionM)/config.ResolutionM) + 2) * config.ResolutionM
	north, east := bounds.Max.NorthM-bounds.Min.NorthM, bounds.Max.EastM-bounds.Min.EastM
	w, h := math.Ceil((east+2*pad)/config.ResolutionM), math.Ceil((north+2*pad)/config.ResolutionM)
	if north <= 0 || east <= 0 || !finite(w*h) || w*h > float64(config.MaxCells) {
		return nil, errors.New("navigation area exceeds grid limits")
	}
	g := &Grid{
		config:         config,
		bounds:         bounds,
		origin:         Point{bounds.Min.NorthM - pad, bounds.Min.EastM - pad},
		width:          int(w),
		height:         int(h),
		lastFrontier:   -1,
		exceptionStart: -1,
		frontierVisits: map[int]int{},
	}
	g.cells = make([]uint8, g.width*g.height)
	g.blocked = make([]bool, len(g.cells))
	for i := range g.blocked {
		g.blocked[i] = true
	}
	return g, nil
}

func (g *Grid) index(p Point) (int, bool) {
	x, y := int(
		math.Floor((p.EastM-g.origin.EastM)/g.config.ResolutionM),
	), int(
		math.Floor((p.NorthM-g.origin.NorthM)/g.config.ResolutionM),
	)
	return y*g.width + x, x >= 0 && y >= 0 && x < g.width && y < g.height
}

func (g *Grid) point(i int) Point {
	p := Point{
		g.origin.NorthM + (float64(i/g.width)+0.5)*g.config.ResolutionM,
		g.origin.EastM + (float64(i%g.width)+0.5)*g.config.ResolutionM,
	}
	half := g.config.ResolutionM / 2
	if p.NorthM < g.bounds.Min.NorthM && p.NorthM+half >= g.bounds.Min.NorthM {
		p.NorthM = g.bounds.Min.NorthM
	}
	if p.NorthM > g.bounds.Max.NorthM && p.NorthM-half <= g.bounds.Max.NorthM {
		p.NorthM = g.bounds.Max.NorthM
	}
	if p.EastM < g.bounds.Min.EastM && p.EastM+half >= g.bounds.Min.EastM {
		p.EastM = g.bounds.Min.EastM
	}
	if p.EastM > g.bounds.Max.EastM && p.EastM-half <= g.bounds.Max.EastM {
		p.EastM = g.bounds.Max.EastM
	}
	return p
}

func (g *Grid) ray(from, to Point) {
	length := distance(from, to)
	steps := max(1, int(math.Ceil(length/(g.config.ResolutionM/3))))
	for i := 0; i <= steps; i++ {
		f := float64(i) / float64(steps)
		p := Point{from.NorthM + (to.NorthM-from.NorthM)*f, from.EastM + (to.EastM-from.EastM)*f}
		if index, ok := g.index(p); ok {
			g.cells[index] = free
		}
	}
}

// Observe интегрирует один новый кадр с компенсацией roll/pitch/yaw и высоты лучей.
// offset переводит мировое положение симулятора в систему старта аппарата.
func (g *Grid) Observe(f *sensors.Frame, offset Point) error {
	if f == nil || f.Validate() != nil {
		return errors.New("invalid navigation sensor frame")
	}
	if g.session != "" && (f.Session != g.session || f.Sequence < g.sequence) {
		return errors.New("navigation sensor session changed")
	}
	if f.Sequence == g.sequence {
		return nil
	}
	g.session, g.sequence = f.Session, f.Sequence
	pose := Point{f.Pose.NorthM + offset.NorthM, f.Pose.EastM + offset.EastM}
	var hits []Point
	l := f.Lidar
	mn, me, mu := f.Pose.Rotate(0, 0, -l.HeightM)
	for layer := range l.Layers {
		elevation := 0.0
		if l.Layers > 1 {
			elevation = l.VerticalFOVRad * (0.5 - float64(layer)/float64(l.Layers-1))
		}
		for i := range l.Resolution {
			bearing := 2 * math.Pi * ((float64(i)+0.5)/float64(l.Resolution) - 0.5)
			if len(l.BearingsRad) > 0 {
				bearing = l.BearingsRad[i]
			}
			n, e, u := f.Pose.Rotate(
				math.Cos(elevation)*math.Cos(bearing),
				math.Cos(elevation)*math.Sin(bearing),
				-math.Sin(elevation),
			)
			r := l.RangesM[layer*l.Resolution+i]
			hit := r != -1
			if !hit {
				r = l.MaxRangeM
			}
			// Лучи, вышедшие из высотного сечения корпуса, не очищают пространство за землёй/кроной.
			if math.Abs(mu) > g.config.VerticalBandM {
				continue
			}
			if math.Abs(u) > 1e-9 {
				limit := (g.config.VerticalBandM - mu) / u
				if u < 0 {
					limit = (-g.config.VerticalBandM - mu) / u
				}
				if limit < r {
					r = math.Max(0, limit)
					hit = false
				}
			}
			from := Point{pose.NorthM + mn, pose.EastM + me}
			to := Point{from.NorthM + n*r, from.EastM + e*r}
			g.ray(from, to)
			if hit {
				hits = append(hits, to)
			}
		}
	}
	for i, s := range f.Sonars {
		if i >= 8 || s.RangeM >= s.MaxRangeM {
			continue
		}
		mn, me, mu := f.Pose.Rotate(s.ForwardM, s.RightM, -s.UpM)
		// Консервативно отмечаем дугу конуса сонара, не используя пропущенное эхо как свободное место.
		for ray := -3; ray <= 3; ray++ {
			bearing := s.BearingRad + float64(ray)*0.35/6
			n, e, u := f.Pose.Rotate(math.Cos(bearing), math.Sin(bearing), 0)
			if math.Abs(mu+u*s.RangeM)+s.RangeM*math.Sin(0.35/2) <= g.config.VerticalBandM {
				hits = append(hits, Point{pose.NorthM + mn + n*s.RangeM, pose.EastM + me + e*s.RangeM})
			}
		}
	}
	for _, p := range hits {
		if i, ok := g.index(p); ok {
			g.cells[i] = occupied
		}
	}
	g.inflate()
	return nil
}

func (g *Grid) inflate() {
	// Интегральная сумма расширяет и препятствия, и неизвестные клетки за O(число клеток).
	stride := g.width + 1
	sums := make([]int, (g.height+1)*stride)
	for y := range g.height {
		row := 0
		for x := range g.width {
			if g.cells[y*g.width+x] != free {
				row++
			}
			sums[(y+1)*stride+x+1] = sums[y*stride+x+1] + row
		}
	}
	// Запас на обе половины диагонали клетки учитывает дискретизацию луча и положения корпуса.
	clearance := g.config.ClearanceM + math.Sqrt2*g.config.ResolutionM
	radius := int(math.Ceil(clearance / g.config.ResolutionM))
	for i := range g.cells {
		x, y := i%g.width, i/g.width
		if g.cells[i] != free || !g.bounds.contains(g.point(i)) || x-radius < 0 || y-radius < 0 ||
			x+radius >= g.width ||
			y+radius >= g.height {
			g.blocked[i] = true
			continue
		}
		x0, y0, x1, y1 := max(0, x-radius), max(0, y-radius), min(g.width, x+radius+1), min(g.height, y+radius+1)
		bad := sums[y1*stride+x1] - sums[y0*stride+x1] - sums[y1*stride+x0] + sums[y0*stride+x0]
		g.blocked[i] = false
		if bad == 0 {
			continue
		}
		for dy := -radius; dy <= radius && !g.blocked[i]; dy++ {
			for dx := -radius; dx <= radius; dx++ {
				if math.Hypot(float64(dx), float64(dy))*g.config.ResolutionM > clearance {
					continue
				}
				if g.cells[(y+dy)*g.width+x+dx] != free {
					g.blocked[i] = true
					break
				}
			}
		}
	}
}

func (g *Grid) lineClear(a, b Point) bool {
	steps := max(1, int(math.Ceil(distance(a, b)/(g.config.ResolutionM/4))))
	for i := 0; i <= steps; i++ {
		f := float64(i) / float64(steps)
		p := Point{a.NorthM + (b.NorthM-a.NorthM)*f, a.EastM + (b.EastM-a.EastM)*f}
		index, ok := g.index(p)
		if !ok || g.blocked[index] || !g.bounds.contains(p) {
			return false
		}
		if index == g.exceptionStart && !g.footprintClear(p) {
			return false
		}
	}
	return true
}

func (g *Grid) footprintClear(p Point) bool {
	index, ok := g.index(p)
	if !ok || g.cells[index] != free {
		return false
	}
	r := int(math.Ceil(g.config.ClearanceM/g.config.ResolutionM)) + 1
	x, y := index%g.width, index/g.width
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			nx, ny := x+dx, y+dy
			if nx < 0 || ny < 0 || nx >= g.width || ny >= g.height {
				return false
			}
			if g.cells[ny*g.width+nx] == free {
				continue
			}
			cn := g.origin.NorthM + (float64(ny)+0.5)*g.config.ResolutionM
			ce := g.origin.EastM + (float64(nx)+0.5)*g.config.ResolutionM
			n := math.Max(0, math.Abs(p.NorthM-cn)-g.config.ResolutionM/2)
			e := math.Max(0, math.Abs(p.EastM-ce)-g.config.ResolutionM/2)
			if math.Hypot(n, e) < g.config.ClearanceM {
				return false
			}
		}
	}
	return true
}
