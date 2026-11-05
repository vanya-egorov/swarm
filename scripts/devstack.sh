#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/devstack/config.sh"
source "$script_dir/devstack/processes.sh"
source "$script_dir/devstack/launch.sh"
source "$script_dir/devstack/supervision.sh"

start_stack() {
  local check_count instance completed=0
  local monitor_done=(false false false)
  local monitor_checks=(false false false)
  validate_stack
  local sitl_dir="$run_dir/sitl-I0"
  if [[ -n "$check" ]]; then
    sitl_dir="$run_dir/sitl-arm-check-I0"
  fi
  mkdir -p "$run_dir/logs"
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

  launch_instances
  supervise_stack
}

case "${1:-}" in
  start) start_stack ;;
  arm-check|flight-check|route-check|group-flight-check|group-route-check) check="$1"; start_stack ;;
  stop) stop_stack ;;
  status) show_status ;;
  *) printf 'Использование: %s {start|arm-check|flight-check|route-check|group-flight-check|group-route-check|stop|status}\n' "$0" >&2; exit 2 ;;
esac
