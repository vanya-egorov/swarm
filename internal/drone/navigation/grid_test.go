package navigation

import (
	"errors"
	"math"
	"testing"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/sensors"
)

func rayBox(start Point, north, east float64, box Bounds) float64 {
	lo, hi := 0.0, math.Inf(1)
	for _, axis := range [][4]float64{{start.NorthM, north, box.Min.NorthM, box.Max.NorthM}, {start.EastM, east, box.Min.EastM, box.Max.EastM}} {
		if math.Abs(axis[1]) < 1e-9 {
			if axis[0] < axis[2] || axis[0] > axis[3] {
				return math.Inf(1)
			}
			continue
		}
		a, b := (axis[2]-axis[0])/axis[1], (axis[3]-axis[0])/axis[1]
		if a > b {
			a, b = b, a
		}
		lo, hi = math.Max(lo, a), math.Min(hi, b)
		if lo > hi {
			return math.Inf(1)
		}
	}
	return lo
}

func scanAt(point Point, boxes []Bounds, sequence uint64) *sensors.Frame {
	f := &sensors.Frame{Version: 1, SystemID: 1, Session: "scan", Sequence: sequence, TimeS: float64(sequence) * 0.1,
		ReceivedAt: time.Now(), Pose: sensors.Pose{NorthM: point.NorthM, EastM: point.EastM, UpM: 1.5},
		Lidar: sensors.Lidar{
			Resolution: 360,
			Layers:     1,
			MinRangeM:  0.12,
			MaxRangeM:  12,
			HeightM:    0.18,
			RangesM:    make([]float64, 360),
		}}
	for i := range f.Lidar.RangesM {
		angle := 2 * math.Pi * ((float64(i)+0.5)/360 - 0.5)
		d := math.Inf(1)
		for _, box := range boxes {
			d = math.Min(d, rayBox(point, math.Cos(angle), math.Sin(angle), box))
		}
		if d > 12 {
			d = -1
		}
		f.Lidar.RangesM[i] = d
	}
	for i := range 10 {
		s := sensors.Sonar{RangeM: 6, MaxRangeM: 6}
		if i < 8 {
			s.BearingRad = float64(i) * math.Pi / 4
			s.ForwardM, s.RightM = 0.3*math.Cos(s.BearingRad), 0.3*math.Sin(s.BearingRad)
		}
		if i == 8 {
			s.ElevationRad, s.UpM = math.Pi/2, 0.15
		}
		if i == 9 {
			s.ElevationRad, s.UpM, s.RangeM = -math.Pi/2, -0.04, 1.46
		}
		f.Sonars = append(f.Sonars, s)
	}
	return f
}

func clearance(point Point, box Bounds) float64 {
	n := math.Max(box.Min.NorthM-point.NorthM, math.Max(0, point.NorthM-box.Max.NorthM))
	e := math.Max(box.Min.EastM-point.EastM, math.Max(0, point.EastM-box.Max.EastM))
	return math.Hypot(n, e)
}

func TestOnlineDetourReturnsToCoverageGoal(t *testing.T) {
	config := DefaultConfig()
	bounds := Bounds{Min: Point{0, 0}, Max: Point{12, 10}}
	box := Bounds{Min: Point{5, 3.5}, Max: Point{6, 6.5}}
	grid, err := NewGrid(bounds, config)
	if err != nil {
		t.Fatal(err)
	}
	current, goal := Point{2, 5}, Point{10, 5}
	detoured := false
	for step := uint64(1); step < 250; step++ {
		if err := grid.Observe(scanAt(current, []Bounds{box}, step), Point{}); err != nil {
			t.Fatal(err)
		}
		carrot, err := grid.Carrot(current, goal)
		if err != nil {
			t.Fatalf("step %d at %v: %v", step, current, err)
		}
		d := distance(current, carrot)
		if d > 0.1 {
			f := 0.1 / d
			carrot = Point{
				current.NorthM + (carrot.NorthM-current.NorthM)*f,
				current.EastM + (carrot.EastM-current.EastM)*f,
			}
		}
		current = carrot
		if !bounds.contains(current) || clearance(current, box) < config.ClearanceM {
			t.Fatalf("unsafe detour at %v", current)
		}
		if math.Abs(current.EastM-5) > 1.5 {
			detoured = true
		}
		if distance(current, goal) < 0.1 {
			if !detoured {
				t.Fatal("route did not pass around obstacle")
			}
			return
		}
	}
	t.Fatalf("navigator did not reach original coverage goal: %v", current)
}

func TestUnknownOccupiedAndBlockedGoal(t *testing.T) {
	g, err := NewGrid(Bounds{Max: Point{12, 10}}, DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	start, goal := Point{2, 5}, Point{10, 5}
	if _, err := g.Path(start, goal); !errors.Is(err, ErrNoPath) {
		t.Fatal("unknown space treated as free")
	}
	wall := Bounds{Min: Point{5, -2}, Max: Point{6, 12}}
	if err := g.Observe(scanAt(start, []Bounds{wall}, 1), Point{}); err != nil {
		t.Fatal(err)
	}
	if path, err := g.Path(start, goal); err == nil && g.lineClear(start, path[len(path)-1]) &&
		path[len(path)-1] == goal {
		t.Fatal("path crosses wall")
	}
	if path, err := g.Path(start, Point{5.1, 5}); err == nil && path[len(path)-1] == (Point{5.1, 5}) {
		t.Fatal("obstacle goal accepted")
	}
}

func TestScanHeadingAndMovingObstacle(t *testing.T) {
	g, err := NewGrid(Bounds{Max: Point{10, 10}}, DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	start := Point{5, 5}
	f := scanAt(start, nil, 1)
	f.Pose.YawRad = math.Pi / 2
	f.Lidar.RangesM[180] = 2
	if err := g.Observe(f, Point{}); err != nil {
		t.Fatal(err)
	}
	index, ok := g.index(Point{5 - math.Sin(math.Pi/360)*2, 5 + math.Cos(math.Pi/360)*2})
	if !ok || g.cells[index] != occupied {
		t.Fatal("lidar bearing was not rotated into east")
	}
	if err := g.Observe(scanAt(start, nil, 2), Point{}); err != nil {
		t.Fatal(err)
	}
	if g.cells[index] != free {
		t.Fatal("old moving obstacle was not cleared by a new ray")
	}
	f = scanAt(start, nil, 3)
	f.Session = "restarted"
	if err := g.Observe(f, Point{}); err == nil {
		t.Fatal("map reused after sensor restart")
	}
}

func TestGridMemoryAndNavigationLimits(t *testing.T) {
	if _, err := NewGrid(Bounds{Max: Point{1e9, 1e9}}, DefaultConfig()); err == nil {
		t.Fatal("unbounded map allocation")
	}
	for _, change := range []func(*Config){func(c *Config) { c.ClearanceM = 0.1 }, func(c *Config) { c.MaxSpeedMS = 2 }, func(c *Config) { c.SensorMaxAge = 0 }, func(c *Config) { c.ResolutionM = math.NaN() }} {
		c := DefaultConfig()
		change(&c)
		if c.Validate() == nil {
			t.Fatal("invalid navigation limits accepted")
		}
	}
}

func TestFractionalBoundaryGoalRemainsReachable(t *testing.T) {
	config := DefaultConfig()
	g, err := NewGrid(Bounds{Min: Point{-1.6, -3.6}, Max: Point{7.6, 3.6}}, config)
	if err != nil {
		t.Fatal(err)
	}
	start, goal := Point{}, Point{-1, -3.60000001}
	if err := g.Observe(scanAt(start, nil, 1), Point{}); err != nil {
		t.Fatal(err)
	}
	path, err := g.Path(start, goal)
	if err != nil || path[len(path)-1] != goal {
		t.Fatalf("boundary goal discarded: %v %v", path, err)
	}
}

func TestWideSonarConeDoesNotCreatePhantomHorizontalWall(t *testing.T) {
	g, err := NewGrid(Bounds{Max: Point{12, 12}}, DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	f := scanAt(Point{5, 5}, nil, 1)
	f.Sonars[0].RangeM = 4
	if err := g.Observe(f, Point{}); err != nil {
		t.Fatal(err)
	}
	for _, cell := range g.cells {
		if cell == occupied {
			t.Fatal("far sonar echo with uncertain height projected into flight plane")
		}
	}
	f = scanAt(Point{5, 5}, nil, 2)
	f.Sonars[0].RangeM = 1
	if err := g.Observe(f, Point{}); err != nil {
		t.Fatal(err)
	}
	hits := 0
	for _, cell := range g.cells {
		if cell == occupied {
			hits++
		}
	}
	if hits == 0 {
		t.Fatal("near sonar obstacle ignored")
	}
}
