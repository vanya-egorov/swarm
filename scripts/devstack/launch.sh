#!/usr/bin/env bash

launch_instances() {
  local port id log_suffix
  : > "$children_file"
  for ((instance=0; instance<DRONE_COUNT; instance++)); do
    port=$((MAVLINK_PORT + instance * 10))
    id=$((SYSTEM_ID + instance))
    log_suffix=""
    if (( DRONE_COUNT > 1 )); then
      sitl_dir="$run_dir/sitl-I${instance}"
      if [[ -n "$check" ]]; then sitl_dir="$run_dir/sitl-arm-check-I${instance}"; fi
      log_suffix="-I${instance}"
    fi
    mkdir -p "$sitl_dir"
    local monitor_args=(-listen "127.0.0.1:${port}" -system-id "$id" -heartbeat-timeout "$HEARTBEAT_TIMEOUT")
    if [[ -n "$check" && ( "$CHECK_INSTANCE" == all || "$CHECK_INSTANCE" == "$instance" ) ]]; then
      monitor_args+=("-sitl-${check}")
      monitor_checks[$instance]=true
    fi
    if [[ "$check" == group-flight-check && "$instance" != 0 ]]; then
      monitor_pids+=("${monitor_pids[0]}")
    else
      "$run_dir/bin/swarm" "${monitor_args[@]}" > "$run_dir/logs/swarm${log_suffix}.log" 2>&1 &
      monitor_pids+=("$!")
    fi

    (
      cd "$sitl_dir"
      exec "$SITL_BIN" \
        --model webots-python \
        --speedup 1 \
        --defaults "$SITL_PARAMS" \
        --sim-address=127.0.0.1 \
        --serial0="udpclient:127.0.0.1:${port}" \
        --sysid "$id" \
        -I"$instance"
    ) > "$run_dir/logs/sitl${log_suffix}.log" 2>&1 &
    sitl_pids+=("$!")
    printf 'instance=%s system_id=%s port=%s sitl=%s monitor=%s\n' \
      "$instance" "$id" "$port" "${sitl_pids[$instance]}" "${monitor_pids[$instance]}" >> "$children_file"
  done
  printf 'Стенд запущен: %s аппарат(а).\n' "$DRONE_COUNT"
  printf 'Логи: %s/logs. Остановка: Ctrl+C или make stop из другого терминала.\n' "$run_dir"

}
