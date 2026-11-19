package coordination

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
)

// HardState сохраняется до выдачи голоса: один голос на срок даже после перезапуска.
type HardState struct {
	ClusterID  string
	Term       uint64
	Generation uint64
	VotedFor   uint8
}

// Store обеспечивает долговечность срока и голоса перед отправкой ответа в сеть.
type Store interface {
	Load() (HardState, error)
	Save(HardState) error
}

// FileStore атомарно сохраняет состояние выборов отдельного агента.
type FileStore struct{ Path string }

// Load читает состояние; отсутствие файла означает первый запуск.
func (s FileStore) Load() (HardState, error) {
	data, err := os.ReadFile(s.Path)
	if errors.Is(err, os.ErrNotExist) {
		return HardState{}, nil
	}
	if err != nil {
		return HardState{}, err
	}
	var state HardState
	if err := decode(data, &state); err != nil {
		return HardState{}, fmt.Errorf("election state: %w", err)
	}
	if state.ClusterID == "" || state.Generation == 0 || (state.Term == 0 && state.VotedFor != 0) {
		return HardState{}, errors.New("incomplete election state")
	}
	return state, nil
}

// Save выполняет fsync файла, атомарную замену и fsync каталога.
func (s FileStore) Save(state HardState) error {
	dir := filepath.Dir(s.Path)
	if err := os.MkdirAll(dir, 0700); err != nil {
		return err
	}
	f, err := os.CreateTemp(dir, ".election-*")
	if err != nil {
		return err
	}
	defer func() { _ = os.Remove(f.Name()) }()
	writeErr := json.NewEncoder(f).Encode(state)
	syncErr := f.Sync()
	closeErr := f.Close()
	if err := errors.Join(writeErr, syncErr, closeErr); err != nil {
		return err
	}
	if err := os.Rename(f.Name(), s.Path); err != nil {
		return err
	}
	directory, err := os.Open(dir)
	if err != nil {
		return err
	}
	return errors.Join(directory.Sync(), directory.Close())
}

func decode(data []byte, target any) error {
	d := json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	if err := d.Decode(target); err != nil {
		return err
	}
	if err := d.Decode(new(any)); !errors.Is(err, io.EOF) {
		return errors.New("unexpected trailing JSON data")
	}
	return nil
}
