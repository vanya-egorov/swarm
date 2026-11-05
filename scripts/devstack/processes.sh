#!/usr/bin/env bash

read_stack_pid() {
  if [[ ! -f "$pid_file" ]]; then
    return 1
  fi
  read -r stack_pid < "$pid_file" || return 1
  [[ "$stack_pid" =~ ^[0-9]+$ ]]
}

stack_is_running() {
  read_stack_pid || return 1
  [[ -p "$control_fifo" && -f "$last_seen_file" ]] || return 1
  local last_seen now
  read -r last_seen < "$last_seen_file" || return 1
  [[ "$last_seen" =~ ^[0-9]+$ ]] || return 1
  now="$(date +%s)"
  (( now >= last_seen && now - last_seen <= 3 ))
}

cleanup() {
  trap - EXIT INT TERM
  local monitor_pid sitl_pid
  for monitor_pid in "${monitor_pids[@]:-}"; do
    [[ -n "$monitor_pid" ]] || continue
    kill -TERM "$monitor_pid" 2>/dev/null || true
    local attempt
    local max_attempts=200
    if [[ "$check" == flight-check || "$check" == route-check || ( "$check" == group-flight-check || "$check" == group-route-check ) ]]; then max_attempts=900; fi
    for ((attempt=0; attempt<max_attempts; attempt++)); do
      kill -0 "$monitor_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$monitor_pid" 2>/dev/null; then
      printf 'Ошибка: Go не завершил очистку за отведённое время; посадка или disarm не подтверждены.\n' >&2
      kill -KILL "$monitor_pid" 2>/dev/null || true
    fi
    wait "$monitor_pid" 2>/dev/null || true
  done
  for sitl_pid in "${sitl_pids[@]:-}"; do
    [[ -n "$sitl_pid" ]] || continue
    kill -TERM "$sitl_pid" 2>/dev/null || true
  done
  for sitl_pid in "${sitl_pids[@]:-}"; do
    [[ -n "$sitl_pid" ]] || continue
    for ((attempt=0; attempt<30; attempt++)); do
      kill -0 "$sitl_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$sitl_pid" 2>/dev/null; then
      printf 'SITL PID %s не ответил на TERM; принудительная остановка.\n' "$sitl_pid" >&2
      kill -KILL "$sitl_pid" 2>/dev/null || true
    fi
    wait "$sitl_pid" 2>/dev/null || true
  done
  exec 3>&- 3<&- || true
  if [[ -f "$pid_file" ]] && [[ "$(<"$pid_file")" == "$$" ]]; then
    rm -f "$pid_file" "$children_file" "$control_fifo" "$last_seen_file"
  fi
  printf 'SITL и Go-монитор остановлены. Webots оставлен открытым.\n'
}

stop_stack() {
  if ! stack_is_running; then
    printf 'Запущенный через make start стенд не найден.\n'
    return 0
  fi
  exec 4<> "$control_fifo"
  printf 'stop\n' >&4
  exec 4>&- 4<&-
  printf 'Команда остановки отправлена стенду (PID %s).\n' "$stack_pid"
}

show_status() {
  if ! stack_is_running; then
    printf 'Стенд не запущен или его состояние нельзя подтвердить.\n'
    return 0
  fi
  printf 'Процесс стенда доступен (PID %s).\n' "$stack_pid"
  if [[ -f "$children_file" ]]; then
    printf 'Дочерние процессы: %s\n' "$(<"$children_file")"
  fi
  printf 'Webots не закрывается автоматически и не учитывается в этом статусе.\n'
}

