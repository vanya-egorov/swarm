package coordination

import (
	"errors"
	"math"
	"math/rand/v2"
	"time"
)

type role uint8

const (
	follower role = iota
	preCandidate
	candidate
	leader
)

type delivery struct {
	to      uint8
	message Message
}

type received struct{ generation, sequence uint64 }

// engine изменяется только в цикле Agent.Run; часы передаются явно для проверки разделений сети.
type engine struct {
	config                                         Config
	store                                          Store
	hard                                           HardState
	role                                           role
	local                                          Status
	leaderID                                       uint8
	leaderSeen, electedAt, deadline, lastBroadcast time.Time
	sequence, round                                uint64
	votes                                          map[uint8]bool
	acks                                           map[uint8]time.Time
	rounds                                         map[uint64]time.Time
	roundACKs                                      map[uint64]map[uint8]bool
	answered                                       map[uint64]time.Time
	releaseUntil                                   time.Time
	seen                                           map[uint8]received
	peers                                          map[uint8]PeerState
	release                                        int
	started                                        bool
	random                                         *rand.Rand
}

func newEngine(config Config, store Store, now time.Time) (*engine, error) {
	hard, err := store.Load()
	if err != nil {
		return nil, err
	}
	if hard.ClusterID != "" && hard.ClusterID != config.ClusterID {
		return nil, errors.New("election store belongs to another mission run")
	}
	if hard.Generation == math.MaxUint64 {
		return nil, errors.New("generation exhausted")
	}
	validVote := hard.VotedFor == 0
	for _, p := range config.Peers {
		validVote = validVote || p.SystemID == hard.VotedFor
	}
	if !validVote {
		return nil, errors.New("stored vote is outside membership")
	}
	hard.ClusterID, hard.Generation = config.ClusterID, hard.Generation+1
	if err := store.Save(hard); err != nil {
		return nil, err
	}
	e := &engine{config: config, store: store, hard: hard,
		votes: map[uint8]bool{}, acks: map[uint8]time.Time{}, rounds: map[uint64]time.Time{},
		seen: map[uint8]received{}, peers: map[uint8]PeerState{},
		roundACKs: map[uint64]map[uint8]bool{}, answered: map[uint64]time.Time{},
		random: rand.New(rand.NewPCG(uint64(config.SystemID), hard.Generation))}
	e.resetDeadline(now)
	return e, nil
}

func (e *engine) resetDeadline(now time.Time) {
	e.deadline = now.Add(
		e.config.ElectionMin + time.Duration(e.random.Int64N(int64(e.config.ElectionMax-e.config.ElectionMin))),
	)
}

func (e *engine) save(term uint64, vote uint8) error {
	state := e.hard
	state.Term, state.VotedFor = term, vote
	if err := e.store.Save(state); err != nil {
		return err
	}
	e.hard = state
	return nil
}

func (e *engine) stepDown(now time.Time) {
	e.role, e.leaderID, e.release = follower, 0, 0
	e.acks = map[uint8]time.Time{}
	e.rounds = map[uint64]time.Time{}
	e.roundACKs, e.answered = map[uint64]map[uint8]bool{}, map[uint64]time.Time{}
	e.releaseUntil = time.Time{}
	e.resetDeadline(now)
}

func (e *engine) message(kind messageKind) Message {
	e.sequence++
	return Message{Version: 1, ClusterID: e.config.ClusterID, From: e.config.SystemID,
		Term: e.hard.Term, Generation: e.hard.Generation, Sequence: e.sequence, Kind: kind, Status: e.local}
}

func (e *engine) broadcast(message Message) []delivery {
	var out []delivery
	for _, peer := range e.config.Peers {
		if peer.SystemID != e.config.SystemID {
			out = append(out, delivery{peer.SystemID, message})
		}
	}
	return out
}

func (e *engine) tick(now time.Time, local Status) ([]delivery, error) {
	e.local = local
	e.started = e.started || local.Started
	e.peers[e.config.SystemID] = PeerState{e.config.SystemID, local, now}
	var out []delivery
	if !local.Healthy {
		if e.role != follower || e.leaderID == e.config.SystemID {
			e.stepDown(now)
		}
		e.resetDeadline(now)
	} else if e.role != leader && !now.Before(e.deadline) {
		if e.hard.Term == math.MaxUint64 {
			return nil, errors.New("election term exhausted")
		}
		e.role, e.leaderID, e.release = preCandidate, 0, 0
		e.votes = map[uint8]bool{e.config.SystemID: true}
		e.resetDeadline(now)
		request := e.message(preVoteRequest)
		request.Round = e.hard.Term + 1
		out = append(out, e.broadcast(request)...)
		if len(e.votes) >= len(e.config.Peers)/2+1 {
			votes, err := e.startElection(now)
			if err != nil {
				return nil, err
			}
			out = append(out, votes...)
		}
	}
	if e.role == leader && !e.hasQuorum(now) && now.Sub(e.electedAt) >= e.config.LeaseTimeout {
		e.stepDown(now)
	}
	if e.lastBroadcast.IsZero() || now.Sub(e.lastBroadcast) >= e.config.HeartbeatInterval {
		e.lastBroadcast = now
		out = append(out, e.broadcast(e.message(statusMessage))...)
		if e.role == leader {
			e.round++
			e.rounds[e.round] = now
			e.roundACKs[e.round] = map[uint8]bool{}
			for round, sent := range e.rounds {
				if now.Sub(sent) >= e.config.LeaseTimeout {
					delete(e.rounds, round)
					delete(e.roundACKs, round)
				}
			}
			e.release = e.nextRelease(now)
			m := e.message(heartbeat)
			m.Round, m.Release = e.round, e.release
			m.LeaseRound = e.leaseRound(now)
			out = append(out, e.broadcast(m)...)
		}
	}
	return out, nil
}

func (e *engine) startElection(now time.Time) ([]delivery, error) {
	if e.hard.Term == math.MaxUint64 {
		return nil, errors.New("election term exhausted")
	}
	if err := e.save(e.hard.Term+1, e.config.SystemID); err != nil {
		return nil, err
	}
	e.role = candidate
	e.votes = map[uint8]bool{e.config.SystemID: true}
	e.resetDeadline(now)
	out := e.broadcast(e.message(voteRequest))
	e.tryBecomeLeader(now)
	return out, nil
}

func (e *engine) tryBecomeLeader(now time.Time) {
	if e.role == candidate && len(e.votes) >= len(e.config.Peers)/2+1 {
		e.role, e.leaderID, e.electedAt = leader, e.config.SystemID, now
		e.acks, e.rounds = map[uint8]time.Time{}, map[uint64]time.Time{}
		e.roundACKs = map[uint64]map[uint8]bool{}
		e.lastBroadcast = time.Time{}
	}
}

func (e *engine) handle(now time.Time, m Message) ([]delivery, error) {
	if !m.validate(e.config) || m.Generation == 0 || m.Sequence == 0 {
		return nil, nil
	}
	seen := e.seen[m.From]
	if m.Generation < seen.generation || (m.Generation == seen.generation && m.Sequence <= seen.sequence) {
		return nil, nil
	}
	e.seen[m.From] = received{m.Generation, m.Sequence}
	if m.Generation > seen.generation {
		delete(e.acks, m.From)
		for _, acknowledgements := range e.roundACKs {
			delete(acknowledgements, m.From)
		}
	}
	previous := e.peers[m.From].Status
	if m.Status.Progress < previous.Progress || (previous.Started && !m.Status.Started) {
		// Перезапущенный исполнитель не может откатить уже подтверждённый прогресс.
		m.Status.Healthy, m.Status.Ready = false, false
		m.Status.Progress, m.Status.Started = previous.Progress, previous.Started
		m.Status.Fault = "execution progress regressed"
	}
	e.peers[m.From] = PeerState{m.From, m.Status, now}
	e.started = e.started || m.Status.Started
	// Предварительное голосование не увеличивает срок изолированного меньшинства.
	if m.Kind == preVoteRequest {
		response := e.message(preVoteResponse)
		response.Round = m.Round
		response.Granted = m.Status.Healthy && m.Round > e.hard.Term && e.role != leader &&
			(e.leaderSeen.IsZero() || now.Sub(e.leaderSeen) >= e.config.ElectionMin)
		return []delivery{{m.From, response}}, nil
	}
	if m.Kind == preVoteResponse {
		if e.role == preCandidate && m.Granted && m.Round == e.hard.Term+1 {
			e.votes[m.From] = true
			if len(e.votes) >= len(e.config.Peers)/2+1 {
				return e.startElection(now)
			}
		}
		return nil, nil
	}
	if m.Term < e.hard.Term {
		return nil, nil
	}
	if m.Term > e.hard.Term {
		if err := e.save(m.Term, 0); err != nil {
			return nil, err
		}
		e.stepDown(now)
	}
	switch m.Kind {
	case voteRequest:
		granted := m.Status.Healthy && (e.hard.VotedFor == 0 || e.hard.VotedFor == m.From) &&
			(e.leaderSeen.IsZero() || now.Sub(e.leaderSeen) >= e.config.LeaseTimeout)
		if granted {
			if err := e.save(m.Term, m.From); err != nil {
				return nil, err
			}
			e.resetDeadline(now)
		}
		response := e.message(voteResponse)
		response.Granted = granted
		return []delivery{{m.From, response}}, nil
	case voteResponse:
		if e.role == candidate && m.Granted {
			e.votes[m.From] = true
			e.tryBecomeLeader(now)
		}
	case heartbeat:
		if !m.Status.Healthy || m.Round == 0 || (e.leaderID != 0 && e.leaderID != m.From) {
			return nil, nil
		}
		if e.leaderID != m.From {
			e.answered = map[uint64]time.Time{}
		}
		e.role, e.leaderID, e.leaderSeen, e.release = follower, m.From, now, 0
		e.releaseUntil = time.Time{}
		if answered, ok := e.answered[m.LeaseRound]; ok && now.Sub(answered) < e.config.LeaseTimeout {
			e.release, e.releaseUntil = m.Release, answered.Add(e.config.LeaseTimeout)
		}
		// Разрешение действительно лишь для недавно подтверждённого этим агентом раунда.
		e.answered[m.Round] = now
		for round, answered := range e.answered {
			if now.Sub(answered) >= e.config.LeaseTimeout {
				delete(e.answered, round)
			}
		}
		e.resetDeadline(now)
		response := e.message(heartbeatACK)
		response.Round = m.Round
		return []delivery{{m.From, response}}, nil
	case heartbeatACK:
		if e.role == leader && m.Status.Healthy {
			if sent, ok := e.rounds[m.Round]; ok && now.Sub(sent) < e.config.LeaseTimeout &&
				sent.After(e.acks[m.From]) {
				e.acks[m.From] = sent
				e.roundACKs[m.Round][m.From] = true
			}
		}
	case statusMessage:
		if m.From == e.leaderID && !m.Status.Healthy {
			e.stepDown(now)
		}
	}
	return nil, nil
}

func (e *engine) hasQuorum(now time.Time) bool {
	if e.role != leader || !e.local.Healthy {
		return false
	}
	return len(e.config.Peers) == 1 || e.leaseRound(now) != 0
}

func (e *engine) leaseRound(now time.Time) uint64 {
	var latest uint64
	for round, sent := range e.rounds {
		if now.Sub(sent) >= e.config.LeaseTimeout {
			continue
		}
		votes := 1
		for id := range e.roundACKs[round] {
			if e.peers[id].Status.Healthy {
				votes++
			}
		}
		if votes >= len(e.config.Peers)/2+1 && round > latest {
			latest = round
		}
	}
	return latest
}

func (e *engine) nextRelease(now time.Time) int {
	if !e.hasQuorum(now) {
		return 0
	}
	progress := e.config.Stages
	active := 0
	for _, member := range e.config.Peers {
		p, known := e.peers[member.SystemID]
		fresh := known && now.Sub(p.ReceivedAt) < e.config.PeerTimeout
		if !e.started && (!fresh || !p.Status.Healthy || !p.Status.Ready) {
			return 0
		}
		if !fresh || !p.Status.Healthy {
			continue
		}
		if !p.Status.Ready {
			return 0
		}
		progress = min(progress, p.Status.Progress)
		active++
	}
	if active < len(e.config.Peers)/2+1 {
		return 0
	}
	return progress + 1
}

func (e *engine) snapshot(now time.Time) Snapshot {
	s := Snapshot{Term: e.hard.Term, LeaderID: e.leaderID, Release: e.release, Quorum: e.hasQuorum(now)}
	if s.Quorum {
		s.ValidUntil = e.rounds[e.leaseRound(now)].Add(e.config.LeaseTimeout)
		if len(e.config.Peers) == 1 {
			s.ValidUntil = now.Add(e.config.LeaseTimeout)
		}
	}
	if e.role != leader && now.Before(e.releaseUntil) && e.local.Healthy {
		s.Quorum = e.release > 0
		s.ValidUntil = e.releaseUntil
	}
	if !s.Quorum {
		s.Release = 0
	}
	for _, peer := range e.config.Peers {
		if p, ok := e.peers[peer.SystemID]; ok {
			s.Peers = append(s.Peers, p)
		}
	}
	return s
}
