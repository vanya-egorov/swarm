package flight

import (
	"context"
	"errors"
	"log/slog"
	"reflect"
	"testing"
	"time"

	"github.com/vanya-egorov/swarm/internal/drone/telemetry"
)

type armingControllerFunc func(context.Context, bool) error

// SetArmed вызывает тестовую реализацию управления.
func (f armingControllerFunc) SetArmed(ctx context.Context, armed bool) error { return f(ctx, armed) }

type stateReaderFunc func() State

// Snapshot возвращает тестовое состояние аппарата.
func (f stateReaderFunc) Snapshot() State { return f() }

func TestArmCheckCases(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name                                    string
		initialArmed, unchangedPWM, cancelOnArm bool
		wantFailure                             bool
		mode                                    FlightMode
		altitude                                float64
		armErr, cleanupErr, wantErr             error
		wantCalls                               []bool
	}{
		{name: "success", wantCalls: []bool{true, false}},
		{name: "armRejected", armErr: ErrCommandRejected, wantErr: ErrCommandRejected, wantCalls: []bool{true, false}},
		{name: "cleanupRejected", cleanupErr: ErrCommandRejected, wantErr: ErrCommandRejected, wantCalls: []bool{true, false}},
		{name: "cancelled", cancelOnArm: true, wantErr: context.Canceled, wantCalls: []bool{true, false}},
		{name: "pwmUnchanged", unchangedPWM: true, wantErr: context.DeadlineExceeded, wantCalls: []bool{true, false}},
		{name: "alreadyArmed", initialArmed: true, wantFailure: true},
		{name: "unknownMode", mode: ModeUnknown, wantFailure: true},
		{name: "aboveGround", altitude: 1, wantFailure: true},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			armed := test.initialArmed
			var calls []bool
			controller := armingControllerFunc(func(commandCtx context.Context, value bool) error {
				if err := commandCtx.Err(); err != nil {
					t.Fatalf("cleanup received cancelled context: %v", err)
				}
				calls = append(calls, value)
				if value && test.armErr != nil {
					return test.armErr
				}
				if !value && test.cleanupErr != nil {
					return test.cleanupErr
				}
				armed = value
				if value && test.cancelOnArm {
					cancel()
					return context.Canceled
				}
				return nil
			})
			states := stateReaderFunc(func() State {
				now := time.Now()
				mode := ModeStabilize
				if test.mode != "" {
					mode = test.mode
				}
				pwm := uint16(1000)
				if armed && !test.unchangedPWM {
					pwm = 1100
				}
				return State{SystemID: 1, ComponentID: 1, Armed: armed, Mode: mode, LastHeartbeat: now,
					Position: &Position{
						SystemID:          1,
						ComponentID:       1,
						RelativeAltitudeM: test.altitude,
						ReceivedAt:        now,
					},
					ServoOutputs: &telemetry.ServoOutputs{
						SystemID:    1,
						ComponentID: 1,
						PWMUS:       [16]uint16{pwm, pwm, pwm, pwm},
						ReceivedAt:  now,
					}}
			})
			config := ArmCheckConfig{
				ReadyTimeout:       time.Second,
				CommandTimeout:     time.Second,
				ObservationTimeout: 60 * time.Millisecond,
				HeartbeatMaxAge:    time.Second,
			}
			err := RunArmCheck(ctx, controller, states, slog.New(slog.DiscardHandler), config)
			if (test.wantErr != nil || test.wantFailure) != (err != nil) {
				t.Fatalf("RunArmCheck() = %v", err)
			}
			if (test.armErr != nil || test.cleanupErr != nil || test.cancelOnArm || test.unchangedPWM) &&
				!errors.Is(err, test.wantErr) {
				t.Fatalf("RunArmCheck() = %v, want %v", err, test.wantErr)
			}
			if !reflect.DeepEqual(calls, test.wantCalls) {
				t.Fatalf("commands = %v, want %v", calls, test.wantCalls)
			}
		})
	}
}

func TestArmCheckTelemetry(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithTimeout(context.Background(), 50*time.Millisecond)
	defer cancel()
	controller := armingControllerFunc(
		func(context.Context, bool) error { t.Fatal("command without telemetry"); return nil },
	)
	err := RunArmCheck(
		ctx,
		controller,
		stateReaderFunc(func() State { return State{} }),
		slog.Default(),
		DefaultArmCheckConfig(),
	)
	if !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("RunArmCheck() = %v", err)
	}
}

func TestArmCheckGroundChange(t *testing.T) {
	t.Parallel()
	reads := 0
	controller := armingControllerFunc(func(context.Context, bool) error {
		t.Fatal("command sent after ground state changed")
		return nil
	})
	states := stateReaderFunc(func() State {
		reads++
		now := time.Now()
		altitude := 0.0
		if reads > 1 {
			altitude = 1
		}
		return State{SystemID: 1, ComponentID: 1, Mode: ModeStabilize, LastHeartbeat: now,
			Position: &Position{SystemID: 1, ComponentID: 1, RelativeAltitudeM: altitude, ReceivedAt: now},
			ServoOutputs: &telemetry.ServoOutputs{
				SystemID:    1,
				ComponentID: 1,
				PWMUS:       [16]uint16{1000, 1000, 1000, 1000},
				ReceivedAt:  now,
			}}
	})
	config := ArmCheckConfig{
		ReadyTimeout:          time.Second,
		StabilizationDuration: 80 * time.Millisecond,
		CommandTimeout:        time.Second,
		ObservationTimeout:    time.Second,
		HeartbeatMaxAge:       time.Second,
	}
	if err := RunArmCheck(context.Background(), controller, states, slog.Default(), config); err == nil {
		t.Fatal("ground state change was ignored")
	}
}

func TestGroundObservationClockRates(t *testing.T) {
	t.Parallel()
	for _, rate := range []float64{0.4, 2} {
		t.Run(time.Duration(rate*100).String(), func(t *testing.T) {
			t.Parallel()
			fake := &flightCheckFake{mode: ModeStabilize}
			before := fake.Snapshot()
			started, lastBoot := time.Now(), time.Duration(0)
			states := stateReaderFunc(func() State {
				state := fake.Snapshot()
				lastBoot = time.Duration(float64(time.Since(started)) * rate)
				state.Position.BootTime = lastBoot
				return state
			})
			duration := 40 * time.Millisecond
			if err := ObserveStableGround(context.Background(), states, before, duration, time.Second); err != nil {
				t.Fatal(err)
			}
			if time.Since(started) < duration || lastBoot < duration {
				t.Fatalf("observation ended early: wall=%s autopilot=%s", time.Since(started), lastBoot)
			}
		})
	}
}

func TestGroundObservationStoppedClock(t *testing.T) {
	t.Parallel()
	fake := &flightCheckFake{mode: ModeStabilize}
	err := ObserveStableGround(context.Background(), fake, fake.Snapshot(), 20*time.Millisecond, time.Second)
	if !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("stopped autopilot did not time out: %v", err)
	}
}

func TestGroundObservationClockReset(t *testing.T) {
	t.Parallel()
	fake := &flightCheckFake{mode: ModeStabilize}
	before := fake.Snapshot()
	before.Position.BootTime = time.Minute
	if err := ObserveStableGround(context.Background(), fake, before, time.Second, time.Second); err == nil {
		t.Fatal("autopilot restart was ignored")
	}
}

type flightCheckFake struct {
	armed      bool
	mode       FlightMode
	altitude   float64
	calls      []string
	takeoffErr error
	overshoot  bool
	landErrors []error
}

// Snapshot возвращает свежий снимок тестового аппарата.
func (f *flightCheckFake) Snapshot() State {
	now := time.Now()
	return State{SystemID: 1, ComponentID: 1, Armed: f.armed, Mode: f.mode, LastHeartbeat: now,
		Position: &Position{SystemID: 1, ComponentID: 1, RelativeAltitudeM: f.altitude, ReceivedAt: now},
		ServoOutputs: &telemetry.ServoOutputs{
			SystemID:    1,
			ComponentID: 1,
			PWMUS:       [16]uint16{1000, 1000, 1000, 1000},
			ReceivedAt:  now,
		}}
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
	if len(f.landErrors) > 0 {
		err := f.landErrors[0]
		f.landErrors = f.landErrors[1:]
		if err != nil {
			return err
		}
	}
	f.mode, f.altitude, f.armed = ModeLand, 0, false
	return nil
}

func TestLandingRetriesAfterLinkRecovery(t *testing.T) {
	t.Parallel()
	fake := &flightCheckFake{mode: ModeGuided, armed: true, altitude: 1.5,
		landErrors: []error{telemetry.ErrHeartbeatTimeout}}
	config := DefaultFlightCheckConfig()
	config.LandingTimeout = 2 * time.Second
	before := fake.Snapshot()
	if err := LandAndConfirm(context.Background(), fake, fake, fake, before, config,
		slog.New(slog.DiscardHandler)); err != nil {
		t.Fatal(err)
	}
	if fake.armed || !reflect.DeepEqual(fake.calls, []string{"land", "land"}) {
		t.Fatalf("landing did not recover: armed=%t commands=%v", fake.armed, fake.calls)
	}
}

func TestFlightCheckCases(t *testing.T) {
	t.Parallel()
	config := FlightCheckConfig{
		ReadyTimeout: time.Second, CommandTimeout: time.Second, ClimbTimeout: time.Second,
		HoldDuration: 20 * time.Millisecond, LandingTimeout: time.Second, TelemetryMaxAge: time.Second,
		TargetAltitudeM: 1.5, MaxAltitudeM: 2.5, MaxDistanceM: 5,
	}
	logger := slog.New(slog.DiscardHandler)
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
