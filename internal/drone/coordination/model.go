// Package coordination обеспечивает обмен состояниями, выбор лидера и барьеры этапов задания.
package coordination

import (
	"errors"
	"fmt"
	"math"
	"net/netip"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/coordinates"
)

// Peer задаёт неизменный system ID участника и его UDP-адрес.
type Peer struct {
	SystemID uint8
	Address  string
}

// Config задаёт состав группы, идентификатор запуска и интервалы реальных часов.
type Config struct {
	SystemID                                                               uint8
	ClusterID                                                              string
	Peers                                                                  []Peer
	Stages                                                                 int
	HeartbeatInterval, LeaseTimeout, ElectionMin, ElectionMax, PeerTimeout time.Duration
}

// DefaultConfig задаёт интервалы для локальной сети; адреса и задание заполняет вызывающий код.
func DefaultConfig() Config {
	return Config{HeartbeatInterval: 100 * time.Millisecond, LeaseTimeout: 500 * time.Millisecond,
		ElectionMin: time.Second, ElectionMax: 2 * time.Second, PeerTimeout: 3 * time.Second}
}

// Validate проверяет фиксированный состав группы и соотношения тайм-аутов.
func (c Config) Validate() error {
	if c.SystemID == 0 || c.ClusterID == "" || len(c.ClusterID) > 128 || len(c.Peers) < 1 || len(c.Peers) > 255 ||
		c.Stages < 1 || c.Stages > 100001 {
		return errors.New("invalid coordination identity, membership or stages")
	}
	if c.HeartbeatInterval < 4*time.Millisecond || c.HeartbeatInterval > c.LeaseTimeout/3 ||
		c.LeaseTimeout >= c.ElectionMin || c.ElectionMin >= c.ElectionMax || c.ElectionMax >= c.PeerTimeout {
		return errors.New("expected 3*heartbeat <= lease < election min < election max < peer timeout")
	}
	ids, addresses := [256]bool{}, make(map[netip.AddrPort]bool)
	for _, peer := range c.Peers {
		address, err := netip.ParseAddrPort(peer.Address)
		if err != nil || address.Port() == 0 || address.Addr().IsUnspecified() || address.Addr().IsMulticast() ||
			peer.SystemID == 0 || ids[peer.SystemID] || addresses[address] {
			return fmt.Errorf("invalid or duplicate coordination peer: %+v", peer)
		}
		ids[peer.SystemID], addresses[address] = true, true
	}
	if !ids[c.SystemID] {
		return errors.New("local system ID is absent from membership")
	}
	return nil
}

// Status содержит здоровье, измеренную позицию и подтверждённый прогресс своего дрона.
// Progress — число завершённых этапов; Started и Progress не уменьшаются в течение запуска.
type Status struct {
	Healthy, Ready, Started, Done        bool
	Progress                             int
	PositionValid, Armed                 bool
	LatitudeDeg, LongitudeDeg, AltitudeM float64
	Mode, Fault                          string
}

func (s Status) validate(stages int) error {
	if s.Progress < 0 || s.Progress > stages || (s.Progress > 0 && !s.Started) ||
		(s.Done && (!s.Started || s.Progress != stages)) || len(s.Fault) > 256 || len(s.Mode) > 32 ||
		(s.Healthy && s.Fault != "") || (s.Ready && !s.Healthy) {
		return errors.New("invalid coordination status")
	}
	if !coordinates.Finite(s.LatitudeDeg) || !coordinates.Finite(s.LongitudeDeg) ||
		!coordinates.Finite(s.AltitudeM) || math.Abs(s.LatitudeDeg) > 90 || math.Abs(s.LongitudeDeg) > 180 {
		return errors.New("invalid coordination position")
	}
	return nil
}

// PeerState содержит последнее полученное состояние; возраст вычисляется по местным часам.
type PeerState struct {
	SystemID   uint8
	Status     Status
	ReceivedAt time.Time
}

// Snapshot представляет безопасную копию состояния координации.
type Snapshot struct {
	Term       uint64
	LeaderID   uint8
	Quorum     bool
	Release    int
	ValidUntil time.Time
	Peers      []PeerState
}

type messageKind string

const (
	preVoteRequest  messageKind = "pre-vote-request"
	preVoteResponse messageKind = "pre-vote-response"
	statusMessage   messageKind = "status"
	voteRequest     messageKind = "vote-request"
	voteResponse    messageKind = "vote-response"
	heartbeat       messageKind = "heartbeat"
	heartbeatACK    messageKind = "heartbeat-ack"
)

// Message — версионированный пакет протокола; координаты маршрутов по сети не передаются.
type Message struct {
	Version                                       int
	ClusterID                                     string
	From                                          uint8
	Term, Round, Generation, Sequence, LeaseRound uint64
	Kind                                          messageKind
	Granted                                       bool
	Release                                       int
	Status                                        Status
}

func (m Message) validate(c Config) bool {
	if m.Version != 1 || m.ClusterID != c.ClusterID || m.From == c.SystemID ||
		m.Release < 0 || m.Release > c.Stages+1 || m.Status.validate(c.Stages) != nil {
		return false
	}
	switch m.Kind {
	case statusMessage, preVoteRequest, preVoteResponse, voteRequest, voteResponse, heartbeat, heartbeatACK:
	default:
		return false
	}
	for _, peer := range c.Peers {
		if peer.SystemID == m.From {
			return true
		}
	}
	return false
}
