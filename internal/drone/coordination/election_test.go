package coordination

import (
	"errors"
	"fmt"
	"testing"
	"time"
)

type memoryStore struct {
	state HardState
	err   error
}

func (s *memoryStore) Load() (HardState, error) { return s.state, s.err }
func (s *memoryStore) Save(state HardState) error {
	if s.err != nil {
		return s.err
	}
	s.state = state
	return nil
}

func testConfig(id uint8) Config {
	c := DefaultConfig()
	c.SystemID, c.ClusterID, c.Stages = id, "test-run", 4
	c.HeartbeatInterval, c.LeaseTimeout = 10*time.Millisecond, 40*time.Millisecond
	c.ElectionMin, c.ElectionMax, c.PeerTimeout = 100*time.Millisecond, 180*time.Millisecond, 250*time.Millisecond
	for i := range 3 {
		c.Peers = append(c.Peers, Peer{uint8(i + 1), fmt.Sprintf("127.0.0.1:%d", 18001+i)})
	}
	return c
}

type cluster struct {
	t       *testing.T
	now     time.Time
	nodes   map[uint8]*engine
	local   map[uint8]Status
	stopped map[uint8]bool
	blocked map[[2]uint8]bool
}

func newCluster(t *testing.T) *cluster {
	t.Helper()
	c := &cluster{t: t, now: time.Unix(1000, 0), nodes: map[uint8]*engine{}, local: map[uint8]Status{},
		stopped: map[uint8]bool{}, blocked: map[[2]uint8]bool{}}
	for id := uint8(1); id <= 3; id++ {
		e, err := newEngine(testConfig(id), &memoryStore{}, c.now)
		if err != nil {
			t.Fatal(err)
		}
		c.nodes[id], c.local[id] = e, Status{Healthy: true, Ready: true}
	}
	return c
}

func (c *cluster) advance(duration time.Duration) {
	c.t.Helper()
	for elapsed := time.Duration(0); elapsed < duration; elapsed += 5 * time.Millisecond {
		c.now = c.now.Add(5 * time.Millisecond)
		var queue []delivery
		for id := uint8(1); id <= 3; id++ {
			if c.stopped[id] {
				continue
			}
			out, err := c.nodes[id].tick(c.now, c.local[id])
			if err != nil {
				c.t.Fatal(err)
			}
			queue = append(queue, out...)
		}
		for len(queue) > 0 {
			d := queue[0]
			queue = queue[1:]
			if c.stopped[d.to] || c.blocked[[2]uint8{d.message.From, d.to}] {
				continue
			}
			out, err := c.nodes[d.to].handle(c.now, d.message)
			if err != nil {
				c.t.Fatal(err)
			}
			queue = append(queue, out...)
		}
		leaders := 0
		for id, e := range c.nodes {
			if !c.stopped[id] && e.snapshot(c.now).Quorum && e.role == leader {
				leaders++
			}
		}
		if leaders > 1 {
			c.t.Fatal("multiple leaders hold quorum simultaneously")
		}
	}
}

func (c *cluster) leader() uint8 {
	c.t.Helper()
	for id, e := range c.nodes {
		if !c.stopped[id] && e.role == leader && e.hasQuorum(c.now) {
			return id
		}
	}
	c.t.Fatal("no leader with quorum")
	return 0
}

func TestElectionFailoverAndProgress(t *testing.T) {
	for _, failure := range []string{"stopped", "unhealthy", "partitioned"} {
		t.Run(failure, func(t *testing.T) {
			c := newCluster(t)
			c.advance(time.Second)
			old := c.leader()
			if c.nodes[old].nextRelease(c.now) != 1 {
				t.Fatal("ready group did not receive launch")
			}
			for id := range c.local {
				c.local[id] = Status{Healthy: true, Ready: true, Started: true, Progress: 2}
			}
			c.advance(50 * time.Millisecond)
			switch failure {
			case "stopped":
				c.stopped[old] = true
			case "unhealthy":
				s := c.local[old]
				s.Healthy, s.Ready, s.Fault = false, false, "motor failure"
				c.local[old] = s
			case "partitioned":
				for id := range c.nodes {
					if id != old {
						c.blocked[[2]uint8{old, id}], c.blocked[[2]uint8{id, old}] = true, true
					}
				}
			}
			c.advance(time.Second)
			next := c.leader()
			if next == old || c.nodes[next].hard.Term <= c.nodes[old].hard.Term && failure == "stopped" {
				t.Fatal("leader was not replaced by a higher-term agent")
			}
			if got := c.nodes[next].nextRelease(c.now); got != 3 {
				t.Fatalf("progress lost: release=%d", got)
			}
			if failure == "partitioned" {
				if s := c.nodes[old].snapshot(c.now); s.Quorum || s.Release != 0 {
					t.Fatal("isolated leader still issues stages")
				}
				clear(c.blocked)
				c.advance(time.Second)
				if c.leader() != next {
					t.Fatal("old leader preempted the new leader after healing")
				}
			}
		})
	}
}

func TestBarrierWaitsForAllBeforeLaunchAndSlowMember(t *testing.T) {
	c := newCluster(t)
	c.local[3] = Status{Healthy: true}
	c.advance(time.Second)
	if c.nodes[c.leader()].nextRelease(c.now) != 0 {
		t.Fatal("launch allowed without all ready")
	}
	c.local[3] = Status{Healthy: true, Ready: true}
	c.advance(50 * time.Millisecond)
	if c.nodes[c.leader()].nextRelease(c.now) != 1 {
		t.Fatal("launch not released")
	}
	for id := range c.local {
		c.local[id] = Status{Healthy: true, Ready: true, Started: true, Progress: 2}
	}
	c.local[3] = Status{Healthy: true, Ready: true, Started: true, Progress: 1}
	c.advance(50 * time.Millisecond)
	if c.nodes[c.leader()].nextRelease(c.now) != 2 {
		t.Fatal("slow member was skipped")
	}
	c.stopped[3] = true
	c.advance(50 * time.Millisecond)
	if c.nodes[c.leader()].nextRelease(c.now) != 2 {
		t.Fatal("missing member skipped before timeout")
	}
	c.advance(time.Second)
	if c.nodes[c.leader()].nextRelease(c.now) != 3 {
		t.Fatal("remaining majority did not continue")
	}
}

func TestMinorityCannotReleaseStages(t *testing.T) {
	c := newCluster(t)
	c.advance(time.Second)
	keep := c.leader()
	for id := range c.nodes {
		if id != keep {
			c.stopped[id] = true
		}
	}
	c.advance(time.Second)
	for id, e := range c.nodes {
		if !c.stopped[id] && (e.hasQuorum(c.now) || e.nextRelease(c.now) != 0) {
			t.Fatal("minority retained control")
		}
	}
}

func TestDurableVoteAndGeneration(t *testing.T) {
	store := &memoryStore{}
	now := time.Unix(100, 0)
	e, err := newEngine(testConfig(1), store, now)
	if err != nil {
		t.Fatal(err)
	}
	e.local = Status{Healthy: true}
	m := Message{Version: 1, ClusterID: "test-run", From: 2, Term: 7, Generation: 1, Sequence: 1,
		Kind: voteRequest, Status: Status{Healthy: true}}
	out, err := e.handle(now, m)
	if err != nil || len(out) != 1 || !out[0].message.Granted {
		t.Fatalf("first vote: %v %v", out, err)
	}
	restarted, err := newEngine(testConfig(1), store, now)
	if err != nil {
		t.Fatal(err)
	}
	if restarted.hard.Generation != 2 {
		t.Fatal("generation not persisted")
	}
	m.From = 3
	out, err = restarted.handle(now, m)
	if err != nil || len(out) != 1 || out[0].message.Granted {
		t.Fatal("double vote after restart")
	}
	store.err = errors.New("disk failure")
	m.Term, m.Sequence = 8, 2
	if _, err := restarted.handle(now, m); err == nil {
		t.Fatal("vote granted without durable storage")
	}
}

func TestDelayedACKAndStalePackets(t *testing.T) {
	c := newCluster(t)
	c.advance(time.Second)
	e := c.nodes[c.leader()]
	peer := uint8(1)
	if peer == e.config.SystemID {
		peer = 2
	}
	oldRound := e.round
	c.now = c.now.Add(e.config.LeaseTimeout + time.Millisecond)
	m := Message{Version: 1, ClusterID: e.config.ClusterID, From: peer, Term: e.hard.Term, Generation: 1,
		Sequence: 10000, Round: oldRound, Kind: heartbeatACK, Status: Status{Healthy: true, Ready: true}}
	if _, err := e.handle(c.now, m); err != nil {
		t.Fatal(err)
	}
	if e.hasQuorum(c.now) {
		t.Fatal("delayed acknowledgement renewed expired lease")
	}
	m.Sequence--
	m.Status.Progress, m.Status.Started = 3, true
	if _, err := e.handle(c.now, m); err != nil {
		t.Fatal(err)
	}
	if e.peers[peer].Status.Progress != 0 {
		t.Fatal("out of order status accepted")
	}
	m.Sequence += 2
	m.ClusterID = "other-run"
	if _, err := e.handle(c.now, m); err != nil {
		t.Fatal(err)
	}
	if e.peers[peer].Status.Progress != 0 {
		t.Fatal("foreign mission accepted")
	}
}
