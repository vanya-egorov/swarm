package vehicle

import (
	"bytes"
	"context"
	"errors"
	"io"
	"log/slog"
	"strings"
	"testing"
	"time"
)

type telemetryReaderFunc func(context.Context) (Telemetry, error)

// Next вызывает тестовый источник телеметрии.
func (f telemetryReaderFunc) Next(ctx context.Context) (Telemetry, error) {
	return f(ctx)
}

func TestMonitorHeartbeat(t *testing.T) {
	t.Parallel()
	var output bytes.Buffer
	logger := slog.New(slog.NewJSONHandler(&output, nil))
	wantErr := errors.New("telemetry stream lost")
	calls := 0
	reader := telemetryReaderFunc(func(context.Context) (Telemetry, error) {
		calls++
		if calls == 1 {
			return Heartbeat{SystemID: 7, ComponentID: 1, Armed: false, ReceivedAt: time.Unix(1, 0)}, nil
		}
		return nil, wantErr
	})
	monitor, err := NewMonitor(reader, logger, time.Second)
	if err != nil {
		t.Fatal(err)
	}

	err = monitor.Run(context.Background())
	if !errors.Is(err, wantErr) {
		t.Fatalf("Run() error = %v, want wrapped error %v", err, wantErr)
	}
	if !strings.Contains(output.String(), `"systemId":7`) {
		t.Fatalf("heartbeat was not logged: %s", output.String())
	}
}

func TestMonitorTimeout(t *testing.T) {
	t.Parallel()
	reader := telemetryReaderFunc(func(ctx context.Context) (Telemetry, error) {
		<-ctx.Done()
		return nil, ctx.Err()
	})
	monitor, err := NewMonitor(reader, slog.Default(), 20*time.Millisecond)
	if err != nil {
		t.Fatal(err)
	}

	if err := monitor.Run(context.Background()); !errors.Is(err, ErrHeartbeatTimeout) {
		t.Fatalf("Run() error = %v, want ErrHeartbeatTimeout", err)
	}
}

func TestMonitorCancel(t *testing.T) {
	t.Parallel()
	reader := telemetryReaderFunc(func(ctx context.Context) (Telemetry, error) {
		<-ctx.Done()
		return nil, ctx.Err()
	})
	monitor, err := NewMonitor(reader, slog.Default(), time.Second)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	if err := monitor.Run(ctx); err != nil {
		t.Fatalf("Run() error = %v, want clean shutdown", err)
	}
}

func TestMonitorPosition(t *testing.T) {
	t.Parallel()
	wantErr := errors.New("stream closed")
	position := Position{SystemID: 7, ComponentID: 1, LatitudeDeg: 55.75, LongitudeDeg: 37.62, RelativeAltitudeM: 12.5}
	events := []Telemetry{
		Heartbeat{SystemID: 7, ComponentID: 1, Armed: true, ReceivedAt: time.Unix(2, 0)},
		position,
	}
	reader := telemetryReaderFunc(func(context.Context) (Telemetry, error) {
		if len(events) == 0 {
			return nil, wantErr
		}
		event := events[0]
		events = events[1:]
		return event, nil
	})
	var output bytes.Buffer
	monitor, err := NewMonitor(reader, slog.New(slog.NewJSONHandler(&output, nil)), time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if err := monitor.Run(context.Background()); !errors.Is(err, wantErr) {
		t.Fatalf("Run() error = %v, want wrapped error %v", err, wantErr)
	}
	snapshot := monitor.Snapshot()
	if snapshot.SystemID != 7 || !snapshot.Armed || snapshot.Position == nil || snapshot.Position.LatitudeDeg != 55.75 {
		t.Fatalf("unexpected state: %+v", snapshot)
	}
	snapshot.Position.LatitudeDeg = 0
	if got := monitor.Snapshot().Position.LatitudeDeg; got != 55.75 {
		t.Fatalf("snapshot changed monitor state: got latitude %v", got)
	}
	if !strings.Contains(output.String(), `"position"`) || !strings.Contains(output.String(), `"latitudeDeg":55.75`) {
		t.Fatalf("position was not logged with units: %s", output.String())
	}
}

func TestMonitorServo(t *testing.T) {
	t.Parallel()
	wantErr := errors.New("stream closed")
	events := []Telemetry{
		Heartbeat{SystemID: 7, ComponentID: 1, ReceivedAt: time.Now()},
		ServoOutputs{SystemID: 7, ComponentID: 1, Port: 1, PWMUS: [16]uint16{1100, 1200, 1300, 1400, 15: 1600}, ReceivedAt: time.Unix(3, 0)},
	}
	reader := telemetryReaderFunc(func(context.Context) (Telemetry, error) {
		if len(events) == 0 {
			return nil, wantErr
		}
		event := events[0]
		events = events[1:]
		return event, nil
	})
	var output bytes.Buffer
	monitor, err := NewMonitor(reader, slog.New(slog.NewJSONHandler(&output, nil)), time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if err := monitor.Run(context.Background()); !errors.Is(err, wantErr) {
		t.Fatalf("Run() error = %v, want wrapped stream error", err)
	}
	state := monitor.Snapshot()
	if state.ServoOutputs == nil || state.ServoOutputs.Port != 1 || state.ServoOutputs.PWMUS != ([16]uint16{1100, 1200, 1300, 1400, 15: 1600}) || state.ServoOutputs.ReceivedAt != time.Unix(3, 0) {
		t.Fatalf("unexpected servo output state: %+v", state)
	}
	state.ServoOutputs.PWMUS[0] = 0
	if got := monitor.Snapshot().ServoOutputs.PWMUS[0]; got != 1100 {
		t.Fatalf("snapshot changed monitor state: got PWM %d", got)
	}
	if !strings.Contains(output.String(), `"servo outputs"`) || !strings.Contains(output.String(), `"pwmUs1To4":[1100,1200,1300,1400]`) {
		t.Fatalf("servo outputs were not logged with units: %s", output.String())
	}
}

func TestMonitorConcurrentSnapshots(t *testing.T) {
	t.Parallel()
	wantErr := errors.New("stream closed")
	events := make(chan Telemetry)
	reader := telemetryReaderFunc(func(ctx context.Context) (Telemetry, error) {
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case event, ok := <-events:
			if !ok {
				return nil, wantErr
			}
			return event, nil
		}
	})
	monitor, err := NewMonitor(reader, slog.New(slog.NewTextHandler(io.Discard, nil)), 5*time.Second)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- monitor.Run(ctx) }()
	producerDone := make(chan struct{})
	go func() {
		defer close(producerDone)
		defer close(events)
		for value := uint16(1000); value > 0; value-- {
			outputs := ServoOutputs{SystemID: 7, ComponentID: 1, ReceivedAt: time.Now()}
			for i := range outputs.PWMUS {
				outputs.PWMUS[i] = value
			}
			select {
			case events <- outputs:
			case <-ctx.Done():
				return
			}
		}
		select {
		case events <- ServoOutputs{SystemID: 7, ComponentID: 1, Port: 1, ReceivedAt: time.Now()}:
		case <-ctx.Done():
		}
	}()
	defer func() {
		cancel()
		<-producerDone
	}()
	for {
		select {
		case err := <-done:
			if !errors.Is(err, wantErr) {
				t.Fatalf("Run() error = %v, want wrapped stream error", err)
			}
			outputs := monitor.Snapshot().ServoOutputs
			if outputs == nil || outputs.Port != 1 || outputs.PWMUS != ([16]uint16{}) {
				t.Fatalf("latest zero-valued port was not stored: %+v", outputs)
			}
			return
		default:
			outputs := monitor.Snapshot().ServoOutputs
			if outputs == nil {
				continue
			}
			for _, pwm := range outputs.PWMUS {
				if pwm != outputs.PWMUS[0] {
					t.Fatalf("snapshot contains mixed updates: %v", outputs.PWMUS)
				}
			}
			outputs.PWMUS[0] = 65535
		}
	}
}

func TestMonitorDeadline(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name  string
		event Telemetry
	}{
		{"position", Position{SystemID: 7, ComponentID: 1}},
		{"servoOutputs", ServoOutputs{SystemID: 7, ComponentID: 1}},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			first := true
			reader := telemetryReaderFunc(func(ctx context.Context) (Telemetry, error) {
				if first {
					first = false
					return Heartbeat{SystemID: 7, ComponentID: 1, ReceivedAt: time.Now()}, nil
				}
				select {
				case <-ctx.Done():
					return nil, ctx.Err()
				case <-time.After(5 * time.Millisecond):
					return test.event, nil
				}
			})
			monitor, err := NewMonitor(reader, slog.New(slog.NewTextHandler(io.Discard, nil)), 40*time.Millisecond)
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			if err := monitor.Run(ctx); !errors.Is(err, ErrHeartbeatTimeout) {
				t.Fatalf("Run() error = %v, want ErrHeartbeatTimeout despite %s updates", err, test.name)
			}
		})
	}
}

func TestMonitorSystemID(t *testing.T) {
	t.Parallel()
	for _, test := range []struct {
		name  string
		event Telemetry
	}{
		{"position", Position{SystemID: 8, ComponentID: 1, LatitudeDeg: 55.75}},
		{"servoOutputs", ServoOutputs{SystemID: 8, ComponentID: 1, PWMUS: [16]uint16{1800}}},
		{"zeroServoSystemId", ServoOutputs{ComponentID: 1}},
	} {
		t.Run(test.name, func(t *testing.T) {
			t.Parallel()
			events := []Telemetry{
				Heartbeat{SystemID: 7, ComponentID: 1, ReceivedAt: time.Now()},
				test.event,
			}
			reader := telemetryReaderFunc(func(context.Context) (Telemetry, error) {
				if len(events) == 0 {
					return nil, errors.New("stream closed")
				}
				event := events[0]
				events = events[1:]
				return event, nil
			})
			monitor, err := NewMonitor(reader, slog.New(slog.NewTextHandler(io.Discard, nil)), time.Second)
			if err != nil {
				t.Fatal(err)
			}
			if err := monitor.Run(context.Background()); !errors.Is(err, ErrSystemIDChanged) {
				t.Fatalf("Run() error = %v, want ErrSystemIDChanged", err)
			}
			state := monitor.Snapshot()
			if state.SystemID != 7 || state.Position != nil || state.ServoOutputs != nil {
				t.Fatalf("mixed telemetry changed state: %+v", state)
			}
		})
	}
}

func TestMonitorComponentChange(t *testing.T) {
	t.Parallel()
	wantErr := errors.New("stream closed")
	events := []Telemetry{
		Heartbeat{SystemID: 7, ComponentID: 1, ReceivedAt: time.Now()},
		Position{SystemID: 7, ComponentID: 1, LatitudeDeg: 55.75},
		ServoOutputs{SystemID: 7, ComponentID: 1, PWMUS: [16]uint16{1100}},
		Heartbeat{SystemID: 7, ComponentID: 2, ReceivedAt: time.Now()},
	}
	reader := telemetryReaderFunc(func(context.Context) (Telemetry, error) {
		if len(events) == 0 {
			return nil, wantErr
		}
		event := events[0]
		events = events[1:]
		return event, nil
	})
	monitor, err := NewMonitor(reader, slog.New(slog.NewTextHandler(io.Discard, nil)), time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if err := monitor.Run(context.Background()); !errors.Is(err, wantErr) {
		t.Fatalf("Run() error = %v, want wrapped stream error", err)
	}
	state := monitor.Snapshot()
	if state.ComponentID != 2 || state.Position != nil || state.ServoOutputs != nil {
		t.Fatalf("old component telemetry was retained: %+v", state)
	}
}
