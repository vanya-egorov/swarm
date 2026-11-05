#!/usr/bin/env bash

supervise_stack() {
  while true; do
    date +%s > "$last_seen_file"
    if IFS= read -r -t 1 instruction <&3; then
      if [[ "$instruction" == "stop" ]]; then
        printf 'Получена команда остановки.\n'
        return 0
      fi
    fi
    for ((instance=0; instance<DRONE_COUNT; instance++)); do
      [[ "${monitor_done[$instance]}" == false ]] || continue
      if ! kill -0 "${sitl_pids[$instance]}" 2>/dev/null; then
        printf 'Ошибка: SITL I%s завершился. Проверьте логи.\n' "$instance" >&2
        return 1
      fi
      if ! kill -0 "${monitor_pids[$instance]}" 2>/dev/null; then
        local monitor_status=0
        wait "${monitor_pids[$instance]}" || monitor_status=$?
        if [[ "${monitor_checks[$instance]}" == true && "$monitor_status" == 0 ]]; then
          monitor_done[$instance]=true
          completed=$((completed + 1))
          if (( completed == check_count )); then
            printf 'Все выбранные проверки SITL завершены.\n'
            return 0
          fi
          continue
        fi
        printf 'Ошибка: Go I%s завершился (код %s). Проверьте логи.\n' "$instance" "$monitor_status" >&2
        return 1
      fi
    done
  done
}

