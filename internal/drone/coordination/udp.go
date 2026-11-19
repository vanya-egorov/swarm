package coordination

import (
	"context"
	"encoding/json"
	"errors"
	"net"
	"net/netip"
	"time"
)

const maxPacketSize = 2048

// Transport передаёт пакеты между известными участниками; Receive учитывает отмену контекста.
type Transport interface {
	Send(context.Context, uint8, Message) error
	Receive(context.Context) (Message, error)
	Close() error
}

// UDPTransport использует отдельный сокет агента и проверяет адрес отправителя по membership.
type UDPTransport struct {
	conn  *net.UDPConn
	peers map[uint8]netip.AddrPort
}

// NewUDPTransport открывает локальный адрес из проверенной конфигурации группы.
func NewUDPTransport(config Config) (*UDPTransport, error) {
	if err := config.Validate(); err != nil {
		return nil, err
	}
	peers := make(map[uint8]netip.AddrPort, len(config.Peers))
	for _, peer := range config.Peers {
		peers[peer.SystemID] = netip.MustParseAddrPort(peer.Address)
	}
	conn, err := net.ListenUDP("udp", net.UDPAddrFromAddrPort(peers[config.SystemID]))
	if err != nil {
		return nil, err
	}
	return &UDPTransport{conn: conn, peers: peers}, nil
}

// Send отправляет один ограниченный по размеру пакет из постоянного сокета своего агента.
func (t *UDPTransport) Send(ctx context.Context, id uint8, message Message) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	address, ok := t.peers[id]
	if !ok {
		return errors.New("unknown coordination destination")
	}
	data, err := json.Marshal(message)
	if err != nil {
		return err
	}
	if len(data) > maxPacketSize {
		return errors.New("coordination packet exceeds size limit")
	}
	deadline := time.Now().Add(100 * time.Millisecond)
	if d, ok := ctx.Deadline(); ok && d.Before(deadline) {
		deadline = d
	}
	if err := t.conn.SetWriteDeadline(deadline); err != nil {
		return err
	}
	_, err = t.conn.WriteToUDPAddrPort(data, address)
	return err
}

// Receive пропускает повреждённые пакеты, неизвестные адреса и подмену system ID.
func (t *UDPTransport) Receive(ctx context.Context) (Message, error) {
	buffer := make([]byte, maxPacketSize+1)
	for {
		if err := ctx.Err(); err != nil {
			return Message{}, err
		}
		if err := t.conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond)); err != nil {
			return Message{}, err
		}
		n, source, err := t.conn.ReadFromUDPAddrPort(buffer)
		if err != nil {
			var networkError net.Error
			if errors.As(err, &networkError) && networkError.Timeout() {
				continue
			}
			return Message{}, err
		}
		var message Message
		if n > maxPacketSize || decode(buffer[:n], &message) != nil {
			continue
		}
		expected, known := t.peers[message.From]
		if known && expected.Addr().Unmap() == source.Addr().Unmap() && expected.Port() == source.Port() {
			return message, nil
		}
	}
}

// Close освобождает сокет и прерывает ожидающее чтение.
func (t *UDPTransport) Close() error { return t.conn.Close() }
