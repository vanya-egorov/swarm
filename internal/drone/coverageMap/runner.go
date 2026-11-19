package coverageMap

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
	"github.com/vanya-egorov/swarm/internal/drone/coordination"
	"github.com/vanya-egorov/swarm/internal/drone/flight"
	"github.com/vanya-egorov/swarm/internal/drone/navigation"
	"github.com/vanya-egorov/swarm/internal/drone/route"
	"github.com/vanya-egorov/swarm/internal/drone/sensors"
	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

// Vehicle объединяет локальные зависимости одного исполнителя покрытия.
type Vehicle struct {
	Flight    flight.FlightController
	Arming    flight.ArmingController
	Positions route.PositionController
	States    telemetry.StateReader
	Sensors   sensors.Reader
	Velocity  route.VelocityController
}

// MissionRunner связывает распределённую координацию с локальным полётом.
// StartMarker исключает повторный автоматический взлёт при перезапуске того же задания.
type MissionRunner struct {
	Config      MissionConfig
	SystemID    uint8
	Transport   coordination.Transport
	Store       coordination.Store
	Vehicle     Vehicle
	Logger      *slog.Logger
	StartMarker string
}

type missionSession struct {
	mu            sync.Mutex
	status        coordination.Status
	mission       PreparedMission
	agent         *coordination.Agent
	vehicle       Vehicle
	marker        string
	sensorSession string
}

// Run проверяет задание, ожидает общую готовность и выполняет покрытие с подтверждённой посадкой.
// Отказ лидера сохраняет текущий этап; потеря координации ограничена FailoverTimeout.
func (r MissionRunner) Run(ctx context.Context) error {
	if r.Vehicle.Flight == nil || r.Vehicle.Arming == nil || r.Vehicle.Positions == nil || r.Vehicle.States == nil ||
		r.Transport == nil || r.Store == nil || r.Logger == nil || r.StartMarker == "" {
		return errors.New("coverage mission: missing dependency or start marker")
	}
	if r.Config.Navigation.Enabled && (r.Vehicle.Sensors == nil || r.Vehicle.Velocity == nil) {
		return errors.New("coverage navigation requires lidar, sonars and velocity control")
	}
	m, err := PrepareMission(r.Config, r.SystemID)
	if err != nil {
		return err
	}
	if _, err := os.Stat(r.StartMarker); !errors.Is(err, os.ErrNotExist) {
		return fmt.Errorf("coverage mission cannot reuse flight marker %q", r.StartMarker)
	}
	agent, err := coordination.NewAgent(m.Config.Network, r.Transport, r.Store, r.Logger)
	if err != nil {
		return err
	}
	s := &missionSession{mission: m, agent: agent, vehicle: r.Vehicle, marker: r.StartMarker}
	networkCtx, stopNetwork := context.WithCancel(context.Background())
	flightCtx, stopFlight := context.WithCancelCause(ctx)
	defer stopFlight(nil)
	networkDone := make(chan error, 1)
	go func() {
		err := agent.Run(networkCtx)
		if networkCtx.Err() == nil {
			stopFlight(fmt.Errorf("coverage network: %w", err))
		}
		networkDone <- err
	}()
	defer func() { stopNetwork(); <-networkDone }()
	readyCtx, stopReady := context.WithTimeout(flightCtx, m.Route.Flight.ReadyTimeout)
	before, err := flight.WaitForState(readyCtx, r.Vehicle.States, func(state telemetry.State) bool {
		if !flight.FreshState(state, m.Route.Flight.TelemetryMaxAge) {
			return false
		}
		return !m.Config.Navigation.Enabled ||
			r.Vehicle.Sensors.Snapshot().Fresh(time.Now(), m.Config.Navigation.SensorMaxAge)
	})
	stopReady()
	if err != nil {
		return err
	}
	if before.SystemID != r.SystemID || before.Armed || before.Mode != telemetry.ModeStabilize ||
		!flight.OnGround(before.Position) ||
		coordinates.HorizontalDistance(before.Position.LatitudeDeg, before.Position.LongitudeDeg,
			m.ExpectedLaunch.LatitudeDeg, m.ExpectedLaunch.LongitudeDeg) > m.Route.HorizontalToleranceM {
		return errors.New("coverage mission: drone must be disarmed at its configured launch in STABILIZE")
	}
	positions := r.Vehicle.Positions
	if m.Config.Navigation.Enabled {
		frame := r.Vehicle.Sensors.Snapshot()
		if frame == nil || frame.Validate() != nil ||
			frame.Sonars[8].RangeM < m.Route.Flight.TargetAltitudeM-before.Position.RelativeAltitudeM+m.Config.Navigation.VerticalBandM {
			return errors.New("sensor geometry invalid or ceiling too low for takeoff")
		}
		launch := Point{}
		for _, l := range m.Config.Launches {
			if l.SystemID == r.SystemID {
				launch = l.Position
			}
		}
		margin := m.Config.MinSeparationM/2 + m.Route.HorizontalToleranceM
		bounds := navigation.Bounds{
			Min: navigation.Point{
				NorthM: m.Assignment.Area.Min.NorthM - launch.NorthM + margin,
				EastM:  m.Assignment.Area.Min.EastM - launch.EastM + margin,
			},
			Max: navigation.Point{
				NorthM: m.Assignment.Area.Max.NorthM - launch.NorthM - margin,
				EastM:  m.Assignment.Area.Max.EastM - launch.EastM - margin,
			},
		}
		navigator, err := navigation.NewDriver(
			m.Config.Navigation,
			bounds,
			r.Vehicle.Sensors,
			r.Vehicle.States,
			r.Vehicle.Positions,
			r.Vehicle.Velocity,
			before.Position,
			r.Logger,
		)
		if err != nil {
			return err
		}
		positions = navigator
		s.sensorSession = frame.Session
	}
	if err := s.refresh(); err != nil {
		return err
	}
	monitorCtx, stopMonitor := context.WithCancel(context.Background())
	monitorDone := make(chan struct{})
	go func() {
		defer close(monitorDone)
		ticker := time.NewTicker(m.Config.Network.HeartbeatInterval / 2)
		defer ticker.Stop()
		lastQuorum := time.Now()
		for {
			select {
			case <-monitorCtx.Done():
				return
			case <-ticker.C:
				if err := s.refresh(); err != nil {
					stopFlight(err)
				}
				if agent.Snapshot().Quorum {
					lastQuorum = time.Now()
				}
				s.mu.Lock()
				flying := s.status.Started && !s.status.Done
				s.mu.Unlock()
				if flying && time.Since(lastQuorum) > m.Config.FailoverTimeout {
					err := errors.New("coverage quorum lost during flight")
					s.Failed(err)
					stopFlight(err)
				}
			}
		}
	}()
	defer func() { stopMonitor(); <-monitorDone }()
	err = route.RunCoordinatedRouteCheck(flightCtx, r.Vehicle.Flight, r.Vehicle.Arming, positions,
		r.Vehicle.States, r.Logger, m.Route, s)
	if err != nil {
		s.mu.Lock()
		s.status.Fault = shortFault(err)
		s.mu.Unlock()
		_ = s.refresh()
		// Сеть продолжает работать до завершения посадки внутри RunCoordinatedRouteCheck.
		return errors.Join(err, context.Cause(flightCtx))
	}
	s.mu.Lock()
	s.status.Done = true
	s.mu.Unlock()
	if err := s.refresh(); err != nil {
		return err
	}
	return s.waitForCompletion(flightCtx)
}

func shortFault(err error) string {
	message := err.Error()
	if len(message) > 256 {
		message = message[:256]
	}
	return message
}

func (s *missionSession) refresh() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	state := s.vehicle.States.Snapshot()
	if s.mission.Config.Navigation.Enabled {
		frame := s.vehicle.Sensors.Snapshot()
		if !frame.Fresh(time.Now(), s.mission.Config.Navigation.SensorMaxAge) || frame.Session != s.sensorSession ||
			frame.SystemID != s.mission.Config.Network.SystemID {
			s.status.Fault = "coverage lidar/sonar stream stale or restarted"
			if frame != nil {
				s.status.Fault = fmt.Sprintf("coverage sensor stream stale/restarted: sequence=%d age=%s",
					frame.Sequence, time.Since(frame.ReceivedAt).Round(time.Millisecond))
			}
		}
	}
	fresh := flight.FreshState(state, s.mission.Route.Flight.TelemetryMaxAge) &&
		state.SystemID == s.mission.Config.Network.SystemID
	if fresh {
		p := state.Position
		s.status.LatitudeDeg, s.status.LongitudeDeg, s.status.AltitudeM = p.LatitudeDeg, p.LongitudeDeg, p.RelativeAltitudeM
		s.status.PositionValid, s.status.Armed, s.status.Mode = true, state.Armed, string(state.Mode)
		if !inside(s.mission.mapPoint(p), s.mission.Assignment.Area, s.mission.Config.MinSeparationM/2) ||
			p.RelativeAltitudeM > s.mission.Route.Flight.MaxAltitudeM || p.RelativeAltitudeM < -0.3 {
			s.status.Fault = "coverage flight bounds exceeded"
		}
		if s.status.Started && s.status.Progress < s.mission.Config.Network.Stages && state.Mode == telemetry.ModeLand {
			s.status.Fault = "coverage flight aborted before final stage"
		}
	} else {
		s.status.PositionValid = false
		s.status.Fault = "coverage telemetry stale or identity changed"
	}
	for _, peer := range s.agent.Snapshot().Peers {
		if peer.SystemID == state.SystemID || !fresh || !peer.Status.PositionValid ||
			time.Since(peer.ReceivedAt) >= s.mission.Config.Network.PeerTimeout {
			continue
		}
		if coordinates.HorizontalDistance(s.status.LatitudeDeg, s.status.LongitudeDeg,
			peer.Status.LatitudeDeg, peer.Status.LongitudeDeg) < s.mission.Config.MinSeparationM {
			s.status.Fault = "coverage minimum separation violated"
		}
	}
	s.status.Healthy, s.status.Ready = fresh && s.status.Fault == "", fresh && s.status.Fault == ""
	if err := s.agent.Update(s.status); err != nil {
		return err
	}
	if s.status.Fault != "" {
		return errors.New(s.status.Fault)
	}
	return nil
}

// Failed исключает неисправный аппарат из лидерства до завершения независимой посадки.
func (s *missionSession) Failed(err error) {
	s.mu.Lock()
	s.status.Fault = shortFault(err)
	s.mu.Unlock()
	_ = s.refresh()
}

// Wait разрешает этап и ограничивает ожидание при смене лидера и потере большинства.
func (s *missionSession) Wait(ctx context.Context, stage int) error {
	waitCtx, cancel := context.WithTimeout(ctx, s.mission.Config.BarrierTimeout)
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- s.agent.Wait(waitCtx, stage) }()
	ticker := time.NewTicker(s.mission.Config.Network.HeartbeatInterval)
	defer ticker.Stop()
	lastQuorum := time.Now()
	for {
		select {
		case err := <-done:
			if err != nil {
				return fmt.Errorf("coverage stage %d: %w", stage, err)
			}
			if stage == 1 {
				if err := s.markStarted(); err != nil {
					return err
				}
			}
			return nil
		case <-waitCtx.Done():
			cancel()
			<-done
			return waitCtx.Err()
		case <-ticker.C:
			if s.agent.Snapshot().Quorum {
				lastQuorum = time.Now()
			}
			if stage > 1 && time.Since(lastQuorum) > s.mission.Config.FailoverTimeout {
				cancel()
				<-done
				return errors.New("coverage coordination lost")
			}
		}
	}
}

func (s *missionSession) markStarted() error {
	if err := s.createMarker(); err != nil {
		return err
	}
	s.mu.Lock()
	s.status.Started = true
	s.mu.Unlock()
	return s.refresh()
}

func (s *missionSession) createMarker() error {
	if err := os.MkdirAll(filepath.Dir(s.marker), 0700); err != nil {
		return err
	}
	f, err := os.OpenFile(s.marker, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return fmt.Errorf("coverage start marker: %w", err)
	}
	_, writeErr := fmt.Fprintln(f, s.mission.Config.Network.ClusterID)
	err = errors.Join(writeErr, f.Sync(), f.Close())
	if err != nil {
		return err
	}
	dir, err := os.Open(filepath.Dir(s.marker))
	if err != nil {
		return err
	}
	return errors.Join(dir.Sync(), dir.Close())
}

// Reached сохраняет подтверждённый этап без повторного запуска при смене лидера.
func (s *missionSession) Reached(stage int) error {
	s.mu.Lock()
	if stage != s.status.Progress+1 {
		s.mu.Unlock()
		return errors.New("coverage stage out of order")
	}
	s.status.Progress = stage
	s.mu.Unlock()
	return s.refresh()
}

func (s *missionSession) waitForCompletion(ctx context.Context) error {
	ctx, cancel := context.WithTimeout(ctx, s.mission.Config.BarrierTimeout+s.mission.Route.Flight.LandingTimeout)
	defer cancel()
	ticker := time.NewTicker(s.mission.Config.Network.HeartbeatInterval)
	defer ticker.Stop()
	var settled time.Time
	for {
		allSettled := true
		var incomplete []uint8
		snapshot := s.agent.Snapshot()
		peers := make(map[uint8]coordination.PeerState, len(snapshot.Peers))
		for _, peer := range snapshot.Peers {
			peers[peer.SystemID] = peer
		}
		for _, member := range s.mission.Config.Network.Peers {
			peer, known := peers[member.SystemID]
			if !known || time.Since(peer.ReceivedAt) >= s.mission.Config.Network.PeerTimeout || !peer.Status.Healthy {
				incomplete = append(incomplete, member.SystemID)
			} else if !peer.Status.Done {
				allSettled = false
			}
		}
		if allSettled {
			if settled.IsZero() {
				settled = time.Now()
			}
			// Несколько повторов Done дают соседям возможность получить подтверждение посадки.
			if time.Since(settled) >= s.mission.Config.Network.LeaseTimeout {
				if len(incomplete) > 0 {
					return fmt.Errorf("coverage incomplete: unavailable vehicles %v", incomplete)
				}
				return nil
			}
		} else {
			settled = time.Time{}
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}
