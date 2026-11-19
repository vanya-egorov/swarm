package sensors

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net"
	"sync"
	"time"
)

// UDPSource принимает кадры только с заданного адреса контроллера выбранного дрона.
type UDPSource struct {
	conn     *net.UDPConn
	expected *net.UDPAddr
	id       uint8
	mu       sync.RWMutex
	frame    *Frame
}

// NewUDPSource использует отдельные фиксированные порты, не смешивая сенсоры и MAVLink.
func NewUDPSource(listen, source string, id uint8) (*UDPSource, error) {
	local, err := net.ResolveUDPAddr("udp4", listen)
	if err != nil {
		return nil, err
	}
	remote, err := net.ResolveUDPAddr("udp4", source)
	if err != nil {
		return nil, err
	}
	if id == 0 || remote.Port == 0 || remote.IP == nil || remote.IP.IsUnspecified() || !remote.IP.IsLoopback() ||
		!local.IP.IsLoopback() {
		return nil, errors.New("sensor bridge requires concrete loopback addresses and a system ID")
	}
	conn, err := net.ListenUDP("udp4", local)
	if err != nil {
		return nil, err
	}
	return &UDPSource{conn: conn, expected: remote, id: id}, nil
}

// Snapshot копирует данные, включая массивы измерений.
func (s *UDPSource) Snapshot() *Frame { s.mu.RLock(); defer s.mu.RUnlock(); return clone(s.frame) }

// Close прерывает чтение и освобождает порт.
func (s *UDPSource) Close() error { return s.conn.Close() }

// Run отбрасывает чужие, повреждённые, повторные кадры и откат часов/поколения.
// Смена контроллера требует нового запуска задания: накопленная карта относится к старому миру.
func (s *UDPSource) Run(ctx context.Context) error {
	buffer := make([]byte, 8193)
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := s.conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond)); err != nil {
			return err
		}
		n, from, err := s.conn.ReadFromUDP(buffer)
		if err != nil {
			var ne net.Error
			if errors.As(err, &ne) && ne.Timeout() {
				continue
			}
			return err
		}
		if !from.IP.Equal(s.expected.IP) || from.Port != s.expected.Port || n > 8192 {
			continue
		}
		var f Frame
		d := json.NewDecoder(bytes.NewReader(buffer[:n]))
		d.DisallowUnknownFields()
		if d.Decode(&f) != nil || !errors.Is(d.Decode(new(any)), io.EOF) || f.SystemID != s.id || f.Validate() != nil {
			continue
		}
		s.mu.Lock()
		previous := s.frame
		if previous != nil &&
			(f.Session != previous.Session || f.Sequence <= previous.Sequence || f.TimeS <= previous.TimeS) {
			s.mu.Unlock()
			continue
		}
		f.ReceivedAt = time.Now()
		s.frame = &f
		s.mu.Unlock()
	}
}
