package vehicle

import (
	"context"
	"errors"
	"io"
	"log/slog"
	"reflect"
	"testing"
	"time"
)

type flightCheckFake struct {
	armed      bool
	mode       FlightMode
	altitude   float64
	calls      []string
	takeoffErr error
	overshoot  bool
}

// Snapshot возвращает свежий снимок тестового аппарата.
func (f *flightCheckFake) Snapshot() State {
	now := time.Now()
	return State{SystemID: 1, ComponentID: 1, Armed: f.armed, Mode: f.mode, LastHeartbeat: now,
		Position:     &Position{SystemID: 1, ComponentID: 1, RelativeAltitudeM: f.altitude, ReceivedAt: now},
		ServoOutputs: &ServoOutputs{SystemID: 1, ComponentID: 1, PWMUS: [16]uint16{1000, 1000, 1000, 1000}, ReceivedAt: now}}
}

// SetMode имитирует подтверждённое переключение в GUIDED.
func (f *flightCheckFake) SetMode(_ context.Context, mode FlightMode) error {
	f.calls = append(f.calls, "mode")
	f.mode = mode
	return nil
}

// SetArmed имитирует подтверждённое вооружение и разоружение.
func (f *flightCheckFake) SetArmed(_ context.Context, armed bool) error {
	if armed {
		f.calls = append(f.calls, "arm")
	} else {
		f.calls = append(f.calls, "disarm")
	}
	f.armed = armed
	return nil
}

// Takeoff имитирует набор высоты или отказ автопилота.
func (f *flightCheckFake) Takeoff(_ context.Context, altitude float64) error {
	f.calls = append(f.calls, "takeoff")
	if f.takeoffErr == nil {
		f.altitude = altitude
		if f.overshoot {
			f.altitude = 3
		}
	}
	return f.takeoffErr
}

// Land имитирует штатную посадку с автоматическим разоружением.
func (f *flightCheckFake) Land(context.Context) error {
	f.calls = append(f.calls, "land")
	f.mode, f.altitude, f.armed = ModeLand, 0, false
	return nil
}

func TestFlightCheckCases(t *testing.T) {
	t.Parallel()
	config := FlightCheckConfig{
		ReadyTimeout: time.Second, CommandTimeout: time.Second, ClimbTimeout: time.Second,
		HoldDuration: 20 * time.Millisecond, LandingTimeout: time.Second, TelemetryMaxAge: time.Second,
		TargetAltitudeM: 1.5, MaxAltitudeM: 2.5, MaxDistanceM: 5,
	}
	logger := slog.New(slog.NewTextHandler(io.Discard, nil))
	for _, test := range []struct {
		name       string
		takeoffErr error
		overshoot  bool
		want       []string
	}{
		{"success", nil, false, []string{"mode", "arm", "takeoff", "land"}},
		{"takeoffRejected", ErrCommandRejected, false, []string{"mode", "arm", "takeoff", "land"}},
		{"altitudeLimit", nil, true, []string{"mode", "arm", "takeoff", "land"}},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			fake := &flightCheckFake{mode: ModeStabilize, takeoffErr: test.takeoffErr, overshoot: test.overshoot}
			err := RunFlightCheck(context.Background(), fake, fake, fake, logger, config)
			if test.overshoot && err == nil {
				t.Fatal("altitude limit ignored")
			}
			if test.takeoffErr != nil && !errors.Is(err, test.takeoffErr) {
				t.Fatalf("rejection lost: %v", err)
			}
			if test.name == "success" && err != nil {
				t.Fatal(err)
			}
			if !reflect.DeepEqual(fake.calls, test.want) {
				t.Fatalf("commands = %v, want %v", fake.calls, test.want)
			}
			if fake.armed || fake.altitude != 0 {
				t.Fatalf("unsafe final state: %+v", fake)
			}
		})
	}
}

func TestFlightCheckGround(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name     string
		armed    bool
		mode     FlightMode
		altitude float64
	}{
		{"armed", true, ModeStabilize, 0},
		{"mode", false, ModeGuided, 0},
		{"altitude", false, ModeStabilize, 1},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			fake := &flightCheckFake{armed: test.armed, mode: test.mode, altitude: test.altitude}
			config := DefaultFlightCheckConfig()
			config.StableDuration = 0
			err := RunFlightCheck(context.Background(), fake, fake, fake, slog.Default(), config)
			if err == nil || len(fake.calls) != 0 {
				t.Fatalf("unsafe state triggered commands: %v, %v", err, fake.calls)
			}
		})
	}
}

func TestLandingGround(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name     string
		position Position
		want     bool
	}{
		{"hover", Position{RelativeAltitudeM: 0.4}, false},
		{"ground", Position{RelativeAltitudeM: 0.02}, true},
		{"moving", Position{DownVelocityMS: 0.3}, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			if got := landedOnGround(&test.position); got != test.want {
				t.Fatalf("landedOnGround() = %t, want %t", got, test.want)
			}
		})
	}
}
