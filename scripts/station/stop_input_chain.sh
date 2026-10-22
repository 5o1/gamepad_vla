#!/usr/bin/env bash
# Stop only registered input process groups; keep the collection service running.
set -euo pipefail
CODE=$(cd "$(dirname "$0")/../.." && pwd)
if [[ -z ${GAMEPAD_VLA_SITE_CONFIG:-} && -f "$CODE/.local/station.json" ]]; then
  export GAMEPAD_VLA_SITE_CONFIG="$CODE/.local/station.json"
fi
source "$CODE/scripts/site_settings.sh"
LOG_ROOT=${STATION_INPUT_LOG_ROOT:-$HOME/gamepad_vla/logs}
while (($#)); do
  case "$1" in
    --log-root) LOG_ROOT=$2; shift 2 ;;
    -h|--help) echo "Usage: $0 [--log-root DIR]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
LOG_ROOT=${LOG_ROOT/#\~/$HOME}
[[ -f "$LOG_ROOT/m2-current" ]] || { echo "No registered input chain"; exit 0; }
L=$(cat "$LOG_ROOT/m2-current")
[[ -f "$L/pids" ]] || { echo "Missing input PID file: $L/pids" >&2; exit 1; }
while read -r name pid; do
  kill -INT -- "-$pid" 2>/dev/null || true
done < "$L/pids"
for ((attempt=0; attempt<10; attempt++)); do
  running=0
  while read -r name pid; do
    # Ignore zombies, but include launch children even if the group leader exited.
    if ps -eo pgid=,stat= | awk -v group="$pid" '$1 == group && $2 !~ /^Z/ {found=1} END {exit !found}'; then
      running=1
    fi
  done < "$L/pids"
  if (( ! running )); then echo "已停止输入链 $L"; exit 0; fi
  sleep 1
done
echo "输入进程组仍在运行（未停止采集服务）：" >&2
while read -r name pid; do
  ps -eo pid,pgid,stat,args | awk -v group="$pid" '$2 == group && $3 !~ /^Z/'
done < "$L/pids"
exit 1
