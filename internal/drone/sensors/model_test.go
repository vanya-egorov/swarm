package sensors

import (
	"context"
	"encoding/json"
	"errors"
	"math"
	"net"
	"testing"
	"time"
)

func testFrame() Frame {
	f := Frame{Version: 1, SystemID: 1, Session: "test", Sequence: 1, TimeS: 1,
		Lidar: Lidar{
			Resolution: 72,
			Layers:     1,
			MinRangeM:  0.12,
			MaxRangeM:  12,
			HeightM:    0.18,
			RangesM:    make([]float64, 72),
		}}
	for i := range f.Lidar.RangesM {
		f.Lidar.RangesM[i] = -1
	}
	for i := range 10 {
		s := Sonar{RangeM: 6, MaxRangeM: 6}
		if i < 8 {
			s.BearingRad = float64(i) * math.Pi / 4
		}
		if i == 8 {
			s.ElevationRad = math.Pi / 2
		}
		if i == 9 {
			s.ElevationRad = -math.Pi / 2
		}
		f.Sonars = append(f.Sonars, s)
	}
	return f
}

func TestFrameGeometryFreshnessAndIsolation(t *testing.T) {
	f := testFrame()
	if err := f.Validate(); err != nil {
		t.Fatal(err)
	}
	f.ReceivedAt = time.Now()
	if !f.Fresh(time.Now(), time.Second) || f.Fresh(time.Now().Add(2*time.Second), time.Second) {
		t.Fatal("incorrect freshness")
	}
	copyFrame := clone(&f)
	copyFrame.Lidar.RangesM[0] = 2
	copyFrame.Sonars[0].RangeM = 1
	if f.Lidar.RangesM[0] != -1 || f.Sonars[0].RangeM != 6 {
		t.Fatal("frame arrays shared across snapshots")
	}
	n, e, u := (Pose{YawRad: math.Pi / 2}).Rotate(1, 0, 0)
	if math.Abs(n) > 1e-9 || math.Abs(e-1) > 1e-9 || math.Abs(u) > 1e-9 {
		t.Fatal("wrong NED rotation")
	}
	for _, change := range []func(*Frame){func(f *Frame) { f.Lidar.RangesM[0] = math.NaN() }, func(f *Frame) { f.Sonars = f.Sonars[:8] },
		func(f *Frame) { f.Sonars[3].BearingRad = 0 }, func(f *Frame) { f.Pose.YawRad = math.Inf(1) }, func(f *Frame) { f.Lidar.Resolution = 1000000 }} {
		invalid := testFrame()
		change(&invalid)
		if invalid.Validate() == nil {
			t.Fatal("invalid sensor geometry accepted")
		}
	}
}

func TestUDPFramesRejectReplayAndForeignController(t *testing.T) {
	writer, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.ParseIP("127.0.0.1")})
	if err != nil {
		t.Fatal(err)
	}
	defer func() { _ = writer.Close() }()
	source, err := NewUDPSource("127.0.0.1:0", writer.LocalAddr().String(), 1)
	if err != nil {
		t.Fatal(err)
	}
	defer func() { _ = source.Close() }()
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- source.Run(ctx) }()
	send := func(f Frame) {
		t.Helper()
		data, err := json.Marshal(f)
		if err != nil {
			t.Fatal(err)
		}
		if _, err := writer.WriteToUDP(data, source.conn.LocalAddr().(*net.UDPAddr)); err != nil {
			t.Fatal(err)
		}
	}
	f := testFrame()
	send(f)
	deadline := time.Now().Add(time.Second)
	for source.Snapshot() == nil && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	first := source.Snapshot()
	if first == nil {
		t.Fatal("sensor frame not received")
	}
	f.Sequence, f.TimeS = 2, 2
	send(f)
	for time.Now().Before(deadline) {
		if source.Snapshot().Sequence == 2 {
			break
		}
		time.Sleep(time.Millisecond)
	}
	if source.Snapshot().Sequence != 2 {
		t.Fatal("fresh frame rejected")
	}
	for _, bad := range []Frame{testFrame(), func() Frame { b := f; b.SystemID = 2; b.Sequence = 3; b.TimeS = 3; return b }(),
		func() Frame { b := f; b.Session = "restart"; b.Sequence = 3; b.TimeS = 3; return b }(),
		func() Frame { b := f; b.Sequence = 3; b.TimeS = 1; return b }()} {
		send(bad)
	}
	time.Sleep(20 * time.Millisecond)
	if source.Snapshot().Sequence != 2 {
		t.Fatal("replay or foreign sensor session accepted")
	}
	cancel()
	_ = source.Close()
	if err := <-done; !errors.Is(err, context.Canceled) && !errors.Is(err, net.ErrClosed) {
		t.Fatal(err)
	}
}
