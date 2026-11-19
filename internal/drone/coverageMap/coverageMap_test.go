package coverageMap

import (
	"math"
	"reflect"
	"testing"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

func TestGenerateRouteSnake(t *testing.T) {
	area := Area{Min: Point{NorthM: -2, EastM: 4}, Max: Point{NorthM: 8, EastM: 10}}
	config := Config{SwathWidthM: 2, LaneSpacingM: 2, AltitudeM: 1.5}
	points, err := GenerateRoute(area, config)
	if err != nil {
		t.Fatal(err)
	}
	want := []route.Waypoint{
		{NorthM: -1, EastM: 5, AltitudeM: 1.5}, {NorthM: 7, EastM: 5, AltitudeM: 1.5},
		{NorthM: 7, EastM: 7, AltitudeM: 1.5}, {NorthM: -1, EastM: 7, AltitudeM: 1.5},
		{NorthM: -1, EastM: 9, AltitudeM: 1.5}, {NorthM: 7, EastM: 9, AltitudeM: 1.5},
	}
	if !reflect.DeepEqual(points, want) {
		t.Fatalf("snake: got %v, want %v", points, want)
	}
	// Поворот карты и направления должен повернуть весь маршрут.
	config.Direction = EastWest
	rotated, err := GenerateRoute(Area{
		Min: Point{NorthM: area.Min.EastM, EastM: area.Min.NorthM},
		Max: Point{NorthM: area.Max.EastM, EastM: area.Max.NorthM},
	}, config)
	if err != nil {
		t.Fatal(err)
	}
	for i, p := range rotated {
		if p.NorthM != points[i].EastM || p.EastM != points[i].NorthM || p.AltitudeM != config.AltitudeM {
			t.Fatalf("rotated waypoint %d: %v", i, p)
		}
	}
}

func TestPlanPartitionsAndCoversArea(t *testing.T) {
	for _, direction := range []Direction{NorthSouth, EastWest} {
		for _, dimensions := range []Point{{0.2, 0.3}, {1, 1}, {11, 7}, {23.75, 16.3}} {
			for _, spacing := range []float64{0.7, 2} {
				for _, ids := range [][]uint8{{7}, {9, 2, 255}, {1, 2, 3, 4, 5, 6, 7}} {
					area := Area{Min: Point{-17, 12}, Max: Point{-17 + dimensions.NorthM, 12 + dimensions.EastM}}
					config := Config{SwathWidthM: 2, LaneSpacingM: spacing, AltitudeM: 1.5, Direction: direction}
					plan, err := Plan(area, ids, config)
					if err != nil {
						t.Fatal(err)
					}
					if len(plan) != len(ids) {
						t.Fatalf("got %d assignments, want %d", len(plan), len(ids))
					}
					for i, assignment := range plan {
						if assignment.SystemID != ids[i] {
							t.Fatalf("assignment order: got %d, want %d", assignment.SystemID, ids[i])
						}
						checkPartition(t, area, plan, i, direction)
						checkCoverage(t, assignment, config)
					}
					again, err := Plan(area, ids, config)
					if err != nil || !reflect.DeepEqual(plan, again) {
						t.Fatalf("plan is not deterministic: %v", err)
					}
				}
			}
		}
	}
}

func checkPartition(t *testing.T, area Area, plan []Assignment, index int, direction Direction) {
	t.Helper()
	subarea := plan[index].Area
	minCross, maxCross := subarea.Min.EastM, subarea.Max.EastM
	start, end := area.Min.EastM, area.Max.EastM
	previous := start
	if index > 0 {
		previous = plan[index-1].Area.Max.EastM
	}
	if direction == NorthSouth {
		if subarea.Min.NorthM != area.Min.NorthM || subarea.Max.NorthM != area.Max.NorthM {
			t.Fatal("partition changed along-track bounds")
		}
	} else {
		minCross, maxCross = subarea.Min.NorthM, subarea.Max.NorthM
		start, end = area.Min.NorthM, area.Max.NorthM
		previous = start
		if index > 0 {
			previous = plan[index-1].Area.Max.NorthM
		}
		if subarea.Min.EastM != area.Min.EastM || subarea.Max.EastM != area.Max.EastM {
			t.Fatal("partition changed along-track bounds")
		}
	}
	if minCross != previous || (index == len(plan)-1 && maxCross != end) {
		t.Fatal("gap or overlap between partitions")
	}
	if math.Abs((maxCross-minCross)-(end-start)/float64(len(plan))) > 1e-10 {
		t.Fatal("unequal partition sizes")
	}
}

func checkCoverage(t *testing.T, assignment Assignment, config Config) {
	t.Helper()
	area, points := assignment.Area, assignment.Waypoints
	if len(points) == 0 {
		t.Fatal("empty coverage route")
	}
	for i, p := range points {
		// Прямоугольник выпуклый: строго внутренние концы гарантируют внутренние отрезки.
		if p.NorthM <= area.Min.NorthM || p.NorthM >= area.Max.NorthM ||
			p.EastM <= area.Min.EastM || p.EastM >= area.Max.EastM || p.AltitudeM != config.AltitudeM {
			t.Fatalf("waypoint outside region or wrong altitude: %v in %v", p, area)
		}
		if i > 0 {
			before := points[i-1]
			if before == p || (before.NorthM != p.NorthM && before.EastM != p.EastM) {
				t.Fatalf("duplicate point or diagonal segment: %v -> %v", before, p)
			}
		}
	}
	// Проверяем пятно обзора вдоль отрезков на сетке, включая все края и углы.
	for n := range 21 {
		for e := range 21 {
			sample := Point{
				NorthM: area.Min.NorthM + (area.Max.NorthM-area.Min.NorthM)*float64(n)/20,
				EastM:  area.Min.EastM + (area.Max.EastM-area.Min.EastM)*float64(e)/20,
			}
			covered := false
			for i, p := range points {
				before := p
				if i > 0 {
					before = points[i-1]
				}
				north := math.Max(math.Min(before.NorthM, p.NorthM)-sample.NorthM,
					sample.NorthM-math.Max(before.NorthM, p.NorthM))
				east := math.Max(math.Min(before.EastM, p.EastM)-sample.EastM,
					sample.EastM-math.Max(before.EastM, p.EastM))
				if math.Max(north, east) <= config.SwathWidthM/2+1e-10 {
					covered = true
					break
				}
			}
			if !covered {
				t.Fatalf("uncovered point %v in region %v", sample, area)
			}
		}
	}
}

func TestSmallAreaHasSingleWaypoint(t *testing.T) {
	area := Area{Min: Point{-1, 2}, Max: Point{-0.5, 2.2}}
	points, err := GenerateRoute(area, DefaultConfig())
	if err != nil || len(points) != 1 {
		t.Fatalf("single footprint: %v, %v", points, err)
	}
	if points[0].NorthM != -0.75 || points[0].EastM != 2.1 {
		t.Fatal(points)
	}
}

func TestWaypointsFromDroneLaunch(t *testing.T) {
	plan, err := Plan(Area{Min: Point{-1, 1}, Max: Point{1, 3}}, []uint8{1, 2}, DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	for i, assignment := range plan {
		launch := Point{NorthM: float64(i), EastM: float64(i + 1)}
		local, err := assignment.WaypointsFrom(launch)
		if err != nil {
			t.Fatal(err)
		}
		origin := &telemetry.Position{
			LatitudeDeg:  launch.NorthM / coordinates.MetersPerDegree,
			LongitudeDeg: launch.EastM / coordinates.MetersPerDegree,
		}
		targets, err := route.TargetsFrom(origin, local)
		if err != nil {
			t.Fatal(err)
		}
		for j, p := range local {
			want := assignment.Waypoints[j]
			if p.NorthM+launch.NorthM != want.NorthM || p.EastM+launch.EastM != want.EastM ||
				p.AltitudeM != want.AltitudeM {
				t.Fatalf("incorrect local offset: %v from %v", p, launch)
			}
			if math.Abs(targets[j].LatitudeDeg*coordinates.MetersPerDegree-want.NorthM) > 1e-6 ||
				math.Abs(targets[j].LongitudeDeg*coordinates.MetersPerDegree-want.EastM) > 1e-6 {
				t.Fatalf("incorrect geographic target: %v", targets[j])
			}
		}
		local[0].NorthM = 100
		if assignment.Waypoints[0].NorthM == 100 {
			t.Fatal("conversion mutated the plan")
		}
	}
}

func TestInvalidArea(t *testing.T) {
	for _, area := range []Area{
		{}, {Max: Point{1, 0}}, {Min: Point{2, 0}, Max: Point{1, 1}},
		{Min: Point{math.NaN(), 0}, Max: Point{1, 1}},
		{Max: Point{1, math.Inf(1)}},
		{Min: Point{-math.MaxFloat64, 0}, Max: Point{math.MaxFloat64, 1}},
	} {
		if plan, err := Plan(area, []uint8{1}, DefaultConfig()); err == nil || plan != nil {
			t.Fatalf("invalid area accepted: %v, %v", area, err)
		}
		if points, err := GenerateRoute(area, DefaultConfig()); err == nil || points != nil {
			t.Fatalf("invalid route area accepted: %v, %v", area, err)
		}
	}
}

func TestInvalidConfig(t *testing.T) {
	area := Area{Max: Point{10, 10}}
	for _, config := range []Config{
		{}, {SwathWidthM: -1, LaneSpacingM: 1, AltitudeM: 1.5},
		{SwathWidthM: 1, LaneSpacingM: 2, AltitudeM: 1.5},
		{SwathWidthM: 1, LaneSpacingM: 0, AltitudeM: 1.5},
		{SwathWidthM: math.Inf(1), LaneSpacingM: 1, AltitudeM: 1.5},
		{SwathWidthM: 1, LaneSpacingM: math.NaN(), AltitudeM: 1.5},
		{SwathWidthM: 1, LaneSpacingM: 1, AltitudeM: -1},
		{SwathWidthM: 1, LaneSpacingM: 1, AltitudeM: math.NaN()},
		{SwathWidthM: 1, LaneSpacingM: 1, AltitudeM: math.Inf(1)},
		{SwathWidthM: 1, LaneSpacingM: 1, AltitudeM: 1.5, Direction: 2},
	} {
		if plan, err := Plan(area, []uint8{1}, config); err == nil || plan != nil {
			t.Fatalf("invalid config accepted: %v, %v", config, err)
		}
		if points, err := GenerateRoute(area, config); err == nil || points != nil {
			t.Fatalf("invalid route config accepted: %v, %v", config, err)
		}
	}
}

func TestInvalidGroup(t *testing.T) {
	area := Area{Max: Point{1, 1}}
	for _, ids := range [][]uint8{nil, {}, {0}, {1, 1}, make([]uint8, 256)} {
		if plan, err := Plan(area, ids, DefaultConfig()); err == nil || plan != nil {
			t.Fatalf("invalid group accepted: %v, %v", ids, err)
		}
	}
	if regions, err := Decompose(area, []uint8{1}, Direction(2)); err == nil || regions != nil {
		t.Fatal("invalid decomposition direction accepted")
	}
	ids := make([]uint8, 255)
	for i := range ids {
		ids[i] = uint8(i + 1)
	}
	if plan, err := Plan(area, ids, DefaultConfig()); err != nil || len(plan) != 255 {
		t.Fatalf("valid maximum group rejected: %v", err)
	}
}

func TestExcessiveOrUnrepresentableRoute(t *testing.T) {
	config := DefaultConfig()
	config.LaneSpacingM = math.SmallestNonzeroFloat64
	if points, err := GenerateRoute(Area{Max: Point{1, 1}}, config); err == nil || points != nil {
		t.Fatal("unbounded allocation allowed")
	}
	area := Area{Min: Point{1e16, 1e16}, Max: Point{1e16 + 2, 1e16 + 2}}
	if points, err := GenerateRoute(area, DefaultConfig()); err == nil || points != nil {
		t.Fatal("route without representable interior points accepted")
	}
	if plan, err := Plan(area, []uint8{1}, DefaultConfig()); err == nil || plan != nil {
		t.Fatal("partial plan returned for unrepresentable route")
	}
	if regions, err := Decompose(area, []uint8{1, 2, 3}, NorthSouth); err == nil || regions != nil {
		t.Fatal("collapsed region accepted")
	}
	config = DefaultConfig()
	if plan, err := Plan(Area{Max: Point{10, 60000}}, []uint8{1, 2}, config); err == nil || plan != nil {
		t.Fatal("total plan waypoint limit bypassed")
	}
}

func TestWaypointsFromRejectsInvalidCoordinates(t *testing.T) {
	assignment := Assignment{Waypoints: []route.Waypoint{{NorthM: 1, EastM: 2, AltitudeM: 1.5}}}
	for _, origin := range []Point{{math.NaN(), 0}, {0, math.Inf(1)}} {
		if points, err := assignment.WaypointsFrom(origin); err == nil || points != nil {
			t.Fatal("invalid launch coordinates accepted")
		}
	}
	assignment.Waypoints[0].NorthM = math.MaxFloat64
	if points, err := assignment.WaypointsFrom(Point{NorthM: -math.MaxFloat64}); err == nil || points != nil {
		t.Fatal("offset overflow accepted")
	}
}
