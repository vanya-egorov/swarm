#!/usr/bin/env bash

project_dir="$(cd "$script_dir/.." && pwd)"
run_dir="${RUN_DIR:-${project_dir}/projectData/run}"
ardupilot_dir="${ARDUPILOT_DIR:-${project_dir}/projectData/ardupilot}"
default_sitl="${ardupilot_dir}/build/sitl/bin/arducopter"
if [[ -x "$run_dir/bin/arducopter-webots" ]]; then
  default_sitl="$run_dir/bin/arducopter-webots"
fi
SITL_BIN="${SITL_BIN:-${default_sitl}}"
SITL_PARAMS="${SITL_PARAMS:-${ardupilot_dir}/libraries/SITL/examples/Webots_Python/params/iris.parm}"
MAVLINK_PORT="${MAVLINK_PORT:-14560}"
SYSTEM_ID="${SYSTEM_ID:-1}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-20s}"
DRONE_COUNT="${DRONE_COUNT:-1}"
CHECK_INSTANCE="${CHECK_INSTANCE:-all}"
pid_file="${run_dir}/stack.pid"
children_file="${run_dir}/children"
control_fifo="${run_dir}/control.fifo"
last_seen_file="${run_dir}/last-seen"

stack_pid=""
sitl_pids=()
monitor_pids=()
check=""

validate_stack() {
  if [[ ( "$check" == group-flight-check || "$check" == group-route-check ) && ( "$DRONE_COUNT" != 3 || "$CHECK_INSTANCE" != all ) ]]; then
    printf 'Групповой полёт требует три SITL и CHECK_INSTANCE=all.\n' >&2
    return 1
  fi
  [[ "$DRONE_COUNT" == 1 || "$DRONE_COUNT" == 3 ]] || { printf 'DRONE_COUNT должен быть 1 или 3.\n' >&2; return 1; }
  if [[ "$DRONE_COUNT" == 3 && ( "$check" == flight-check || "$check" == route-check ) && "$CHECK_INSTANCE" == all ]]; then
    printf 'Полётные проверки доступны только для одного SITL.\n' >&2
    return 1
  fi
  [[ "$SYSTEM_ID" =~ ^[0-9]+$ && "$MAVLINK_PORT" =~ ^[0-9]+$ ]] || return 1
  SYSTEM_ID=$((10#$SYSTEM_ID))
  MAVLINK_PORT=$((10#$MAVLINK_PORT))
  if (( SYSTEM_ID < 1 || SYSTEM_ID + DRONE_COUNT - 1 > 255 || MAVLINK_PORT < 1024 || MAVLINK_PORT + 10 * (DRONE_COUNT - 1) > 65535 )); then
    printf 'ID или порты выходят за допустимые границы.\n' >&2
    return 1
  fi
  if [[ -n "$check" ]]; then
    SITL_PARAMS="${ARM_CHECK_PARAMS:-${project_dir}/sim/ardupilot/iris-arm-check.parm}"
  fi
  [[ -x "$SITL_BIN" ]] || { printf 'ArduCopter SITL не найден: %s\n' "$SITL_BIN" >&2; return 1; }
  [[ -f "$SITL_PARAMS" ]] || { printf 'Параметры Iris не найдены: %s\n' "$SITL_PARAMS" >&2; return 1; }
  if stack_is_running; then
    printf 'Стенд уже запущен (PID %s). Используйте make status или make stop.\n' "$stack_pid" >&2
    return 1
  fi

  check_count="$DRONE_COUNT"
  if [[ "$CHECK_INSTANCE" != all ]]; then
    [[ "$CHECK_INSTANCE" =~ ^[0-2]$ ]] && (( CHECK_INSTANCE < DRONE_COUNT )) || { printf 'Недопустимый CHECK_INSTANCE.\n' >&2; return 1; }
    check_count=1
  fi

}
