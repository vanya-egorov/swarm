#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-${project_dir}/projectData/run}"
ardupilot_dir="${ARDUPILOT_DIR:-${project_dir}/projectData/ardupilot}"
SITL_BIN="${SITL_BIN:-${ardupilot_dir}/build/sitl/bin/arducopter}"
SITL_PARAMS="${SITL_PARAMS:-${ardupilot_dir}/libraries/SITL/examples/Webots_Python/params/iris.parm}"
MAVLINK_PORT="${MAVLINK_PORT:-14560}"
SYSTEM_ID="${SYSTEM_ID:-1}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-20s}"
pid_file="${run_dir}/stack.pid"
children_file="${run_dir}/children"
control_fifo="${run_dir}/control.fifo"
last_seen_file="${run_dir}/last-seen"

stack_pid=""
sitl_pid=""
monitor_pid=""
arm_check=false
flight_check=false

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
  if [[ -n "$monitor_pid" ]]; then
    kill -TERM "$monitor_pid" 2>/dev/null || true
    local attempt
    local max_attempts=200
    if [[ "$flight_check" == true ]]; then max_attempts=900; fi
    for ((attempt=0; attempt<max_attempts; attempt++)); do
      kill -0 "$monitor_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$monitor_pid" 2>/dev/null; then
      printf 'Ошибка: Go не завершил очистку за отведённое время; посадка или disarm не подтверждены.\n' >&2
      kill -KILL "$monitor_pid" 2>/dev/null || true
    fi
    wait "$monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "$sitl_pid" ]]; then
    kill -TERM "$sitl_pid" 2>/dev/null || true
    wait "$sitl_pid" 2>/dev/null || true
  fi
  exec 3>&- 3<&- || true
  if [[ -f "$pid_file" ]] && [[ "$(<"$pid_file")" == "$$" ]]; then
    rm -f "$pid_file" "$children_file" "$control_fifo" "$last_seen_file"
  fi
  printf 'SITL и Go-монитор остановлены. Webots оставлен открытым.\n'
}

start_stack() {
  if [[ "$arm_check" == true || "$flight_check" == true ]]; then
    SITL_PARAMS="${ARM_CHECK_PARAMS:-${project_dir}/sim/ardupilot/iris-arm-check.parm}"
  fi
  [[ -x "$SITL_BIN" ]] || { printf 'ArduCopter SITL не найден: %s\n' "$SITL_BIN" >&2; return 1; }
  [[ -f "$SITL_PARAMS" ]] || { printf 'Параметры Iris не найдены: %s\n' "$SITL_PARAMS" >&2; return 1; }
  if stack_is_running; then
    printf 'Стенд уже запущен (PID %s). Используйте make status или make stop.\n' "$stack_pid" >&2
    return 1
  fi

  local sitl_dir="$run_dir/sitl-I0"
  if [[ "$arm_check" == true || "$flight_check" == true ]]; then
    sitl_dir="$run_dir/sitl-arm-check-I0"
  fi
  mkdir -p "$run_dir/logs" "$sitl_dir"
  if [[ -e "$control_fifo" ]]; then
    rm -f "$control_fifo"
  fi
  mkfifo "$control_fifo"
  exec 3<> "$control_fifo"
  printf '%s\n' "$$" > "$pid_file"
  date +%s > "$last_seen_file"
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  local monitor_args=(-listen "127.0.0.1:${MAVLINK_PORT}" -system-id "$SYSTEM_ID" -heartbeat-timeout "$HEARTBEAT_TIMEOUT")
  if [[ "$arm_check" == true ]]; then
    monitor_args+=(-sitl-arm-check)
  fi
  if [[ "$flight_check" == true ]]; then
    monitor_args+=(-sitl-flight-check)
  fi
  "$run_dir/bin/swarm" "${monitor_args[@]}" > "$run_dir/logs/swarm.log" 2>&1 &
  monitor_pid=$!

  (
    cd "$sitl_dir"
    exec "$SITL_BIN" \
      --model webots-python \
      --speedup 1 \
      --defaults "$SITL_PARAMS" \
      --sim-address=127.0.0.1 \
      --serial0="udpclient:127.0.0.1:${MAVLINK_PORT}" \
      --sysid "$SYSTEM_ID" \
      -I0
  ) > "$run_dir/logs/sitl.log" 2>&1 &
  sitl_pid=$!

  printf 'sitl=%s monitor=%s\n' "$sitl_pid" "$monitor_pid" > "$children_file"
  printf 'Стенд запущен: SITL PID %s, Go PID %s.\n' "$sitl_pid" "$monitor_pid"
  printf 'Логи: %s/logs. Остановка: Ctrl+C или make stop из другого терминала.\n' "$run_dir"

  while true; do
    date +%s > "$last_seen_file"
    if IFS= read -r -t 1 instruction <&3; then
      if [[ "$instruction" == "stop" ]]; then
        printf 'Получена команда остановки.\n'
        return 0
      fi
    fi
    if ! kill -0 "$sitl_pid" 2>/dev/null; then
      printf 'Ошибка: SITL завершился. Последние сообщения:\n' >&2
      tail -n 15 "$run_dir/logs/sitl.log" >&2 || true
      return 1
    fi
    if ! kill -0 "$monitor_pid" 2>/dev/null; then
      local monitor_status=0
      wait "$monitor_pid" || monitor_status=$?
      if [[ "$arm_check" == true || "$flight_check" == true ]] && [[ "$monitor_status" == 0 ]]; then
        printf 'Проверка SITL завершена.\n'
        return 0
      fi
      printf 'Ошибка: Go завершился (код %s). Последние сообщения:\n' "$monitor_status" >&2
      tail -n 15 "$run_dir/logs/swarm.log" >&2 || true
      return 1
    fi
  done
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

case "${1:-}" in
  start) start_stack ;;
  arm-check) arm_check=true; start_stack ;;
  flight-check) flight_check=true; start_stack ;;
  stop) stop_stack ;;
  status) show_status ;;
  *) printf 'Использование: %s {start|arm-check|flight-check|stop|status}\n' "$0" >&2; exit 2 ;;
esac
