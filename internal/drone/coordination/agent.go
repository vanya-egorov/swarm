package coordination

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"sync"
	"time"
)

// Agent запускает сетевой цикл одного дрона; полётные команды остаются у локального исполнителя.
type Agent struct {
	config    Config
	transport Transport
	engine    *engine
	logger    *slog.Logger
	mu        sync.Mutex
	local     Status
	snapshot  Snapshot
	changed   chan struct{}
	running   bool
	stopped   bool
}

// NewAgent загружает долговечные голоса до запуска сети.
func NewAgent(config Config, transport Transport, store Store, logger *slog.Logger) (*Agent, error) {
	if err := config.Validate(); err != nil {
		return nil, err
	}
	if transport == nil || store == nil || logger == nil {
		return nil, errors.New("missing coordination dependency")
	}
	config.Peers = append([]Peer(nil), config.Peers...)
	e, err := newEngine(config, store, time.Now())
	if err != nil {
		return nil, err
	}
	return &Agent{config: config, transport: transport, engine: e, logger: logger, changed: make(chan struct{})}, nil
}

// Update публикует состояние своего исполнителя, запрещая откат прогресса.
func (a *Agent) Update(status Status) error {
	if err := status.validate(a.config.Stages); err != nil {
		return err
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	if status.Progress < a.local.Progress || (a.local.Started && !status.Started) || (a.local.Done && !status.Done) {
		return errors.New("local coordination progress cannot regress")
	}
	a.local = status
	return nil
}

// Snapshot возвращает копию наблюдаемого состояния группы.
func (a *Agent) Snapshot() Snapshot {
	a.mu.Lock()
	defer a.mu.Unlock()
	s := a.snapshot
	if !a.local.Healthy || !time.Now().Before(s.ValidUntil) {
		s.Quorum, s.Release = false, 0
	}
	s.Peers = append([]PeerState(nil), s.Peers...)
	return s
}

// Wait ожидает разрешения этапа от действующего лидера; повторный вызов не исполняет этап.
func (a *Agent) Wait(ctx context.Context, stage int) error {
	if stage < 1 || stage > a.config.Stages+1 {
		return errors.New("invalid coordination stage")
	}
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		a.mu.Lock()
		s, local, changed, stopped := a.snapshot, a.local, a.changed, a.stopped
		a.mu.Unlock()
		if stopped {
			return errors.New("coordination agent stopped")
		}
		if !local.Healthy {
			return errors.New("local drone is unhealthy")
		}
		if s.Quorum && time.Now().Before(s.ValidUntil) && s.Release >= stage {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-changed:
		}
	}
}

func (a *Agent) publish(now time.Time) {
	next := a.engine.snapshot(now)
	a.mu.Lock()
	previous := a.snapshot
	a.snapshot = next
	close(a.changed)
	a.changed = make(chan struct{})
	a.mu.Unlock()
	if previous.LeaderID != next.LeaderID || previous.Term != next.Term {
		a.logger.Info("coordination leader changed", "systemId", a.config.SystemID,
			"leaderId", next.LeaderID, "term", next.Term, "quorum", next.Quorum)
	}
}

// Run обменивается состояниями до отмены; потеря большинства прекращает выдачу этапов.
// Повторный запуск одного экземпляра запрещён; транспорт закрывается при завершении.
func (a *Agent) Run(ctx context.Context) error {
	a.mu.Lock()
	if a.running {
		a.mu.Unlock()
		return errors.New("coordination agent already used")
	}
	a.running = true
	a.mu.Unlock()
	ctx, cancel := context.WithCancel(ctx)
	packets := make(chan Message, 256)
	receiveDone := make(chan error, 1)
	go func() {
		for {
			message, err := a.transport.Receive(ctx)
			if err != nil {
				receiveDone <- err
				return
			}
			select {
			case packets <- message:
			case <-ctx.Done():
				receiveDone <- ctx.Err()
				return
			}
		}
	}()
	receivedExit := false
	defer func() {
		cancel()
		_ = a.transport.Close()
		if !receivedExit {
			<-receiveDone
		}
		a.mu.Lock()
		a.stopped = true
		a.snapshot.Quorum, a.snapshot.Release = false, 0
		close(a.changed)
		a.changed = make(chan struct{})
		a.mu.Unlock()
	}()
	ticker := time.NewTicker(a.config.HeartbeatInterval / 4)
	defer ticker.Stop()
	for {
		var out []delivery
		var err error
		select {
		case <-ctx.Done():
			return ctx.Err()
		case err = <-receiveDone:
			receivedExit = true
			return fmt.Errorf("coordination receive: %w", err)
		case m := <-packets:
			out, err = a.engine.handle(time.Now(), m)
		case <-ticker.C:
			a.mu.Lock()
			local := a.local
			a.mu.Unlock()
			out, err = a.engine.tick(time.Now(), local)
		}
		if err != nil {
			return fmt.Errorf("coordination state: %w", err)
		}
		for _, packet := range out {
			if err := a.transport.Send(ctx, packet.to, packet.message); err != nil && ctx.Err() == nil {
				a.logger.Debug("coordination packet lost", "peerId", packet.to, "error", err)
			}
		}
		a.publish(time.Now())
	}
}
