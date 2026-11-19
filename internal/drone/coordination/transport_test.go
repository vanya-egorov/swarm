package coordination

import (
	"context"
	"encoding/json"
	"errors"
	"log/slog"
	"net"
	"net/netip"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func TestUDPExchangeFiltersMalformedAndWrongSender(t *testing.T) {
	conns := make([]*net.UDPConn, 3)
	addresses := make(map[uint8]netip.AddrPort)
	for i := range conns {
		conn, err := net.ListenUDP("udp", net.UDPAddrFromAddrPort(netip.MustParseAddrPort("127.0.0.1:0")))
		if err != nil {
			t.Fatal(err)
		}
		conns[i] = conn
		t.Cleanup(func() { _ = conn.Close() })
		addresses[uint8(i+1)] = conn.LocalAddr().(*net.UDPAddr).AddrPort()
	}
	one := &UDPTransport{conn: conns[0], peers: addresses}
	two := &UDPTransport{conn: conns[1], peers: addresses}
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	packet := Message{
		Version:    1,
		ClusterID:  "run",
		From:       1,
		Generation: 1,
		Sequence:   1,
		Kind:       statusMessage,
		Status:     Status{Healthy: true},
	}
	for _, data := range [][]byte{[]byte("not JSON"), make([]byte, maxPacketSize+1), []byte(`{"Unknown":true}`)} {
		if _, err := conns[0].WriteToUDPAddrPort(data, addresses[2]); err != nil {
			t.Fatal(err)
		}
	}
	forged := packet
	forged.From = 3
	encoded, err := json.Marshal(forged)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := conns[0].WriteToUDPAddrPort(encoded, addresses[2]); err != nil {
		t.Fatal(err)
	}
	if err := one.Send(ctx, 2, packet); err != nil {
		t.Fatal(err)
	}
	got, err := two.Receive(ctx)
	if err != nil || got != packet {
		t.Fatalf("UDP exchange: %+v %v", got, err)
	}
	if err := one.Send(ctx, 9, packet); err == nil {
		t.Fatal("unknown recipient accepted")
	}
	cancel()
	if _, err := two.Receive(ctx); !errors.Is(err, context.Canceled) {
		t.Fatal("receive ignores cancellation")
	}
}

func TestUDPConstructorAndClose(t *testing.T) {
	reserve, err := net.ListenUDP("udp", net.UDPAddrFromAddrPort(netip.MustParseAddrPort("127.0.0.1:0")))
	if err != nil {
		t.Fatal(err)
	}
	address := reserve.LocalAddr().String()
	if err := reserve.Close(); err != nil {
		t.Fatal(err)
	}
	c := testConfig(1)
	c.Peers = []Peer{{SystemID: 1, Address: address}}
	transport, err := NewUDPTransport(c)
	if err != nil {
		t.Fatal(err)
	}
	if err := transport.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := transport.Receive(context.Background()); err == nil {
		t.Fatal("closed transport still reads")
	}
}

func TestFileStoreAtomicAndFailClosed(t *testing.T) {
	store := FileStore{Path: filepath.Join(t.TempDir(), "nested", "state.json")}
	if state, err := store.Load(); err != nil || state != (HardState{}) {
		t.Fatal(state, err)
	}
	want := HardState{ClusterID: "mission", Term: 7, Generation: 3, VotedFor: 2}
	if err := store.Save(want); err != nil {
		t.Fatal(err)
	}
	if got, err := store.Load(); err != nil || got != want {
		t.Fatal(got, err)
	}
	want.Term = 8
	if err := store.Save(want); err != nil {
		t.Fatal(err)
	}
	if got, err := store.Load(); err != nil || got != want {
		t.Fatal(got, err)
	}
	for _, content := range []string{"broken", `{}`, `{"Term":8,"Unknown":true}`, `{"Term":8} {}`} {
		if err := os.WriteFile(store.Path, []byte(content), 0600); err != nil {
			t.Fatal(err)
		}
		if _, err := store.Load(); err == nil {
			t.Fatal("corrupt election file ignored")
		}
	}
}

func TestInvalidMembershipAndTimings(t *testing.T) {
	for _, name := range []string{"identity", "duplicate", "address", "unknown", "clock", "zero-tick"} {
		t.Run(name, func(t *testing.T) {
			c := testConfig(1)
			switch name {
			case "identity":
				c.ClusterID = ""
			case "duplicate":
				c.Peers[1] = c.Peers[0]
			case "address":
				c.Peers[0].Address = "0.0.0.0:9000"
			case "unknown":
				c.SystemID = 9
			case "clock":
				c.ElectionMin = c.LeaseTimeout
			case "zero-tick":
				c.HeartbeatInterval = time.Nanosecond
			}
			if c.Validate() == nil {
				t.Fatal("invalid configuration accepted")
			}
		})
	}
}

func TestDelayedHeartbeatCannotReleaseStages(t *testing.T) {
	c := newCluster(t)
	c.advance(time.Second)
	leaderID := c.leader()
	id := uint8(1)
	if id == leaderID {
		id = 2
	}
	e := c.nodes[id]
	if s := e.snapshot(c.now); !s.Quorum || s.Release != 1 {
		t.Fatalf("initial lease missing: %+v", s)
	}
	leaseRound := uint64(0)
	for round := range e.answered {
		leaseRound = max(leaseRound, round)
	}
	c.now = c.now.Add(e.config.LeaseTimeout + time.Millisecond)
	m := Message{Version: 1, ClusterID: e.config.ClusterID, From: leaderID, Term: e.hard.Term, Generation: 1,
		Sequence: 100000, Round: leaseRound + 1, LeaseRound: leaseRound, Release: 4, Kind: heartbeat,
		Status: Status{Healthy: true, Ready: true}}
	if _, err := e.handle(c.now, m); err != nil {
		t.Fatal(err)
	}
	if s := e.snapshot(c.now); s.Quorum || s.Release != 0 {
		t.Fatal("delayed leader command revived expired follower lease")
	}
}

func TestWaitRejectsExpiredPublishedLease(t *testing.T) {
	a := &Agent{config: testConfig(1), local: Status{Healthy: true}, changed: make(chan struct{}),
		snapshot: Snapshot{Quorum: true, Release: 4, ValidUntil: time.Now().Add(-time.Second)}}
	if s := a.Snapshot(); s.Quorum || s.Release != 0 {
		t.Fatal("expired snapshot exposed active permission")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Millisecond)
	defer cancel()
	if err := a.Wait(ctx, 2); !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("expired lease released stage: %v", err)
	}
}

func TestUDPAgentsReplaceStoppedLeader(t *testing.T) {
	config := testConfig(1)
	config.HeartbeatInterval, config.LeaseTimeout = 20*time.Millisecond, 100*time.Millisecond
	config.ElectionMin, config.ElectionMax, config.PeerTimeout = 200*time.Millisecond, 350*time.Millisecond, 500*time.Millisecond
	addresses := map[uint8]netip.AddrPort{}
	connections := map[uint8]*net.UDPConn{}
	config.Peers = nil
	for id := uint8(1); id <= 3; id++ {
		conn, err := net.ListenUDP("udp", net.UDPAddrFromAddrPort(netip.MustParseAddrPort("127.0.0.1:0")))
		if err != nil {
			t.Fatal(err)
		}
		t.Cleanup(func() { _ = conn.Close() })
		address := conn.LocalAddr().(*net.UDPAddr).AddrPort()
		addresses[id], connections[id] = address, conn
		config.Peers = append(config.Peers, Peer{SystemID: id, Address: address.String()})
	}
	ctx, stop := context.WithTimeout(context.Background(), 5*time.Second)
	defer stop()
	agents := map[uint8]*Agent{}
	cancels := map[uint8]context.CancelFunc{}
	done := make(chan error, 3)
	for id := uint8(1); id <= 3; id++ {
		c := config
		c.SystemID = id
		a, err := NewAgent(
			c,
			&UDPTransport{conn: connections[id], peers: addresses},
			&memoryStore{},
			slog.New(slog.DiscardHandler),
		)
		if err != nil {
			t.Fatal(err)
		}
		if err := a.Update(Status{Healthy: true, Ready: true, Started: true, Progress: 2}); err != nil {
			t.Fatal(err)
		}
		localCtx, cancel := context.WithCancel(ctx)
		agents[id], cancels[id] = a, cancel
		go func() { done <- a.Run(localCtx) }()
	}
	defer func() {
		stop()
		for range 3 {
			<-done
		}
	}()
	waitLeader := func(except uint8) uint8 {
		for {
			for id, a := range agents {
				if id != except {
					if s := a.Snapshot(); s.LeaderID == id && s.Quorum && s.Release == 3 {
						return id
					}
				}
			}
			select {
			case <-ctx.Done():
				t.Fatal("UDP cluster did not elect leader")
				return 0
			case <-time.After(10 * time.Millisecond):
			}
		}
	}
	old := waitLeader(0)
	cancels[old]()
	if next := waitLeader(old); next == old {
		t.Fatal("stopped UDP leader retained authority")
	}
}
