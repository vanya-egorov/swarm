package route

import (
	"context"
	"errors"
	"log/slog"
	"reflect"
	"testing"
)

type stageRecorder struct {
	waited, reached []int
	failAt          int
	failure         error
	notified        bool
	vehicle         *routeFake
}

func (c *stageRecorder) Wait(ctx context.Context, stage int) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	c.waited = append(c.waited, stage)
	if stage == c.failAt {
		return c.failure
	}
	return nil
}
func (c *stageRecorder) Reached(stage int) error { c.reached = append(c.reached, stage); return nil }
func (c *stageRecorder) Failed(error) {
	c.notified = len(c.vehicle.calls) > 0 && c.vehicle.calls[len(c.vehicle.calls)-1] != "land"
}

func TestCoordinatedRouteStagesAndFailureLanding(t *testing.T) {
	for _, failAt := range []int{0, 1, 2, 3, 4} {
		f := &routeFake{mode: ModeStabilize}
		wantErr := errors.New("coordinator unavailable")
		c := &stageRecorder{failAt: failAt, failure: wantErr, vehicle: f}
		err := RunCoordinatedRouteCheck(
			context.Background(),
			f,
			f,
			f,
			f,
			slog.New(slog.DiscardHandler),
			routeTestConfig(),
			c,
		)
		if failAt == 0 {
			if err != nil || !reflect.DeepEqual(c.waited, []int{1, 2, 3, 4}) ||
				!reflect.DeepEqual(c.reached, []int{1, 2, 3}) {
				t.Fatalf("incorrect coordinated stages: %v %v %v", c.waited, c.reached, err)
			}
		} else if !errors.Is(err, wantErr) {
			t.Fatalf("barrier failure lost: %v", err)
		}
		if failAt == 1 {
			if len(f.calls) != 0 {
				t.Fatal("commands issued without launch permission")
			}
		} else {
			if f.armed || f.calls[len(f.calls)-1] != "land" {
				t.Fatal("landing missing")
			}
			if failAt > 1 && !c.notified {
				t.Fatal("failure not published before landing")
			}
		}
	}
}
