package coverageMap

import (
	"crypto/sha256"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"slices"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/coordination"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

// MapOrigin задаёт географическое начало общей метрической системы карты.
type MapOrigin struct{ LatitudeDeg, LongitudeDeg float64 }

// Launch задаёт известное место старта участника в системе карты.
type Launch struct {
	SystemID uint8
	Position Point
}

// MissionConfig описывает один неизменный запуск, одинаковый на всех агентах.
type MissionConfig struct {
	RunID                           string
	Area                            Area
	Origin                          MapOrigin
	Coverage                        Config
	Launches                        []Launch
	Network                         coordination.Config
	Route                           route.RouteCheckConfig
	MinSeparationM                  float64
	FailoverTimeout, BarrierTimeout time.Duration
}

// DefaultMissionConfig сохраняет ограничения полётов SITL и задаёт интервалы координации.
func DefaultMissionConfig() MissionConfig {
	return MissionConfig{Coverage: DefaultConfig(), Network: coordination.DefaultConfig(),
		Route: route.DefaultRouteCheckConfig(), MinSeparationM: 4,
		FailoverTimeout: 6 * time.Second, BarrierTimeout: 40 * time.Second}
}

// PreparedMission содержит проверенный локальный маршрут и общую идентичность задания.
type PreparedMission struct {
	Config         MissionConfig
	Assignment     Assignment
	Route          route.RouteCheckConfig
	ExpectedLaunch route.PositionTarget
}

// PrepareMission проверяет маршруты всех участников до открытия сети и любых полётных команд.
// Короткие маршруты дополнены удержанием последней точки для общих барьеров.
func PrepareMission(config MissionConfig, systemID uint8) (PreparedMission, error) {
	if config.RunID == "" || len(config.RunID) > 64 || !coordinates.Finite(config.MinSeparationM) ||
		config.MinSeparationM <= 0 ||
		config.FailoverTimeout <= config.Network.PeerTimeout+config.Network.ElectionMax ||
		config.BarrierTimeout <= config.Route.WaypointTimeout+config.FailoverTimeout {
		return PreparedMission{}, errors.New("invalid mission identity, separation or timing")
	}
	if !coordinates.Finite(config.Origin.LatitudeDeg) || math.Abs(config.Origin.LatitudeDeg) > 85 ||
		!coordinates.Finite(config.Origin.LongitudeDeg) || math.Abs(config.Origin.LongitudeDeg) > 180 {
		return PreparedMission{}, errors.New("invalid mission map origin")
	}
	config.Network.Peers = append([]coordination.Peer(nil), config.Network.Peers...)
	slices.SortFunc(config.Network.Peers, func(a, b coordination.Peer) int { return int(a.SystemID) - int(b.SystemID) })
	config.Launches = append([]Launch(nil), config.Launches...)
	slices.SortFunc(config.Launches, func(a, b Launch) int { return int(a.SystemID) - int(b.SystemID) })
	ids := make([]uint8, len(config.Network.Peers))
	for i, peer := range config.Network.Peers {
		ids[i] = peer.SystemID
	}
	clearance := config.MinSeparationM/2 + config.Route.HorizontalToleranceM
	config.Coverage.BoundaryMarginM = math.Max(config.Coverage.BoundaryMarginM, clearance)
	assignments, err := Plan(config.Area, ids, config.Coverage)
	if err != nil {
		return PreparedMission{}, err
	}
	if len(config.Launches) != len(ids) {
		return PreparedMission{}, errors.New("one launch per member required")
	}
	length := 0
	for _, a := range assignments {
		length = max(length, len(a.Waypoints))
	}
	config.Network.SystemID, config.Network.Stages = systemID, length+1
	// Локальный ID не входит в общий отпечаток задания.
	config.Network.ClusterID, config.Network.SystemID = "", 0
	config.Route.Waypoints = nil
	data, err := json.Marshal(config)
	if err != nil {
		return PreparedMission{}, err
	}
	config.Network.ClusterID = fmt.Sprintf("%x", sha256.Sum256(data))
	config.Network.SystemID = systemID
	if err := config.Network.Validate(); err != nil {
		return PreparedMission{}, err
	}
	var prepared PreparedMission
	for i, assignment := range assignments {
		launch := config.Launches[i]
		if launch.SystemID != assignment.SystemID || !inside(launch.Position, assignment.Area, clearance) {
			return PreparedMission{}, fmt.Errorf(
				"vehicle %d: launch must be inside its region with clearance %.2f m",
				assignment.SystemID,
				clearance,
			)
		}
		for _, p := range assignment.Waypoints {
			if !inside(Point{p.NorthM, p.EastM}, assignment.Area, clearance) {
				return PreparedMission{}, fmt.Errorf(
					"vehicle %d: route violates separation clearance",
					assignment.SystemID,
				)
			}
		}
		points, err := assignment.WaypointsFrom(launch.Position)
		if err != nil {
			return PreparedMission{}, err
		}
		for len(points) < length {
			points = append(points, points[len(points)-1])
		}
		routeConfig := config.Route
		routeConfig.Waypoints = points
		routeConfig.Flight.TargetAltitudeM = config.Coverage.AltitudeM
		if err := routeConfig.Validate(); err != nil {
			return PreparedMission{}, fmt.Errorf("vehicle %d: %w", assignment.SystemID, err)
		}
		origin := &telemetry.Position{LatitudeDeg: config.Origin.LatitudeDeg, LongitudeDeg: config.Origin.LongitudeDeg}
		launchTarget, err := route.TargetsFrom(origin, []route.Waypoint{{NorthM: launch.Position.NorthM,
			EastM: launch.Position.EastM, AltitudeM: config.Coverage.AltitudeM}})
		if err != nil {
			return PreparedMission{}, err
		}
		launchPosition := &telemetry.Position{
			LatitudeDeg:  launchTarget[0].LatitudeDeg,
			LongitudeDeg: launchTarget[0].LongitudeDeg,
		}
		if _, err := route.TargetsFrom(launchPosition, points); err != nil {
			return PreparedMission{}, fmt.Errorf("vehicle %d geographic route: %w", assignment.SystemID, err)
		}
		if assignment.SystemID == systemID {
			prepared = PreparedMission{
				Config:         config,
				Assignment:     assignment,
				Route:          routeConfig,
				ExpectedLaunch: launchTarget[0],
			}
		}
	}
	return prepared, nil
}

func inside(point Point, area Area, margin float64) bool {
	return coordinates.Finite(point.NorthM) && coordinates.Finite(point.EastM) &&
		point.NorthM >= area.Min.NorthM+margin && point.NorthM <= area.Max.NorthM-margin &&
		point.EastM >= area.Min.EastM+margin && point.EastM <= area.Max.EastM-margin
}

func (m PreparedMission) mapPoint(p *telemetry.Position) Point {
	return Point{NorthM: (p.LatitudeDeg - m.Config.Origin.LatitudeDeg) * coordinates.MetersPerDegree,
		EastM: math.Remainder(p.LongitudeDeg-m.Config.Origin.LongitudeDeg, 360) * coordinates.MetersPerDegree *
			math.Cos(m.Config.Origin.LatitudeDeg*math.Pi/180)}
}
