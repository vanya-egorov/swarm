package vehicle

import (
	"context"
	"errors"
	"log/slog"
	"reflect"
	"testing"
	"time"
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
					ServoOutputs: &ServoOutputs{
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
			ServoOutputs: &ServoOutputs{
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
