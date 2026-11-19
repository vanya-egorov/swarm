package navigation

import (
	"container/heap"
	"errors"
	"math"
)

// ErrNoPath означает отсутствие наблюдённого прохода в пределах выделенной области.
var ErrNoPath = errors.New("no observed obstacle-free path inside assigned region")

type node struct {
	index int
	cost  float64
}
type priorityQueue []node

func (q priorityQueue) Len() int           { return len(q) }
func (q priorityQueue) Less(i, j int) bool { return q[i].cost < q[j].cost }
func (q priorityQueue) Swap(i, j int)      { q[i], q[j] = q[j], q[i] }
func (q *priorityQueue) Push(x any)        { *q = append(*q, x.(node)) }
func (q *priorityQueue) Pop() any          { old := *q; n := old[len(old)-1]; *q = old[:len(old)-1]; return n }

func (g *Grid) frontier(i int) bool {
	r := int(math.Ceil(g.config.ClearanceM/g.config.ResolutionM)) + 2
	x, y := i%g.width, i/g.width
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			nx, ny := x+dx, y+dy
			if nx < 0 || ny < 0 || nx >= g.width || ny >= g.height {
				continue
			}
			j := ny*g.width + nx
			if g.cells[j] == unknown && g.bounds.contains(g.point(j)) {
				return true
			}
		}
	}
	return false
}

// Path находит проход по наблюдённому пространству; неизвестную часть исследует через фронтиры.
// Непроходимая или занятая цель возвращает ошибку вместо ложного подтверждения покрытия.
func (g *Grid) Path(start, goal Point) ([]Point, error) {
	s, ok := g.index(start)
	if !ok || !g.bounds.contains(start) || !g.bounds.contains(goal) || g.blocked[s] {
		return nil, ErrNoPath
	}
	target, ok := g.index(goal)
	if !ok || g.cells[target] == occupied {
		return nil, ErrNoPath
	}
	if g.lineClear(start, goal) {
		return []Point{start, goal}, nil
	}
	dist := make([]float64, len(g.cells))
	parent := make([]int, len(g.cells))
	for i := range dist {
		dist[i], parent[i] = math.Inf(1), -1
	}
	dist[s] = 0
	q := priorityQueue{{index: s}}
	heap.Init(&q)
	best, score := -1, math.Inf(1)
	for q.Len() > 0 {
		current := heap.Pop(&q).(node)
		i := current.index
		if current.cost != dist[i] {
			continue
		}
		if i == target {
			best = i
			break
		}
		if current.cost >= g.config.LookaheadM && g.frontier(i) {
			candidate := distance(g.point(i), goal) + current.cost*0.1 + float64(g.frontierVisits[i])*1000
			if candidate < score {
				score, best = candidate, i
			}
		}
		x, y := i%g.width, i/g.width
		for dy := -1; dy <= 1; dy++ {
			for dx := -1; dx <= 1; dx++ {
				if dx == 0 && dy == 0 {
					continue
				}
				nx, ny := x+dx, y+dy
				if nx < 0 || ny < 0 || nx >= g.width || ny >= g.height {
					continue
				}
				j := ny*g.width + nx
				if g.blocked[j] || (dx != 0 && dy != 0 && (g.blocked[y*g.width+nx] || g.blocked[ny*g.width+x])) {
					continue
				}
				step := g.config.ResolutionM * math.Hypot(float64(dx), float64(dy))
				if cost := current.cost + step; cost < dist[j] {
					dist[j], parent[j] = cost, i
					heap.Push(&q, node{j, cost})
				}
			}
		}
	}
	if best < 0 {
		return nil, ErrNoPath
	}
	if g.lastFrontier >= 0 && distance(start, g.point(g.lastFrontier)) < g.config.LookaheadM {
		g.frontierVisits[g.lastFrontier]++
	}
	if best != target {
		g.lastFrontier = best
	} else {
		g.lastFrontier = -1
	}
	var reverse []Point
	for i := best; i != s; i = parent[i] {
		if i < 0 {
			return nil, ErrNoPath
		}
		reverse = append(reverse, g.point(i))
	}
	path := []Point{start}
	for i := len(reverse) - 1; i >= 0; i-- {
		path = append(path, reverse[i])
	}
	if best == target {
		path = append(path, goal)
	}
	return path, nil
}

// Carrot выбирает достижимую короткую цель, не срезая углы препятствий.
func (g *Grid) Carrot(start, goal Point) (Point, error) {
	if index, ok := g.index(start); ok && g.blocked[index] && g.bounds.contains(start) && g.footprintClear(start) {
		// После нового скана граница инфляции может сдвинуться на клетку. Выход разрешён
		// лишь из геометрически проверенного положения в обычные свободные клетки.
		g.blocked[index] = false
		g.exceptionStart = index
		defer func() { g.blocked[index] = true; g.exceptionStart = -1 }()
	}
	path, err := g.Path(start, goal)
	if err != nil {
		return Point{}, err
	}
	next := start
	for _, p := range path[1:] {
		d := distance(start, p)
		candidate := p
		if d > g.config.LookaheadM {
			f := g.config.LookaheadM / d
			candidate = Point{start.NorthM + (p.NorthM-start.NorthM)*f, start.EastM + (p.EastM-start.EastM)*f}
		}
		if !g.lineClear(start, candidate) {
			break
		}
		next = candidate
		if d >= g.config.LookaheadM {
			break
		}
	}
	if distance(start, next) < 1e-6 && distance(start, goal) > g.config.ResolutionM {
		return Point{}, ErrNoPath
	}
	return next, nil
}
