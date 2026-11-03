#!/usr/bin/env bash
# Restart the station collection service (port 8780) with a chosen scene spec.
#   bash scripts/station/restart_collection_server.sh [SPEC]
# SPEC defaults to the current collection scene. Refuses to restart while an
# episode is recording. Starts no hardware input node.
set -euo pipefail
CODE=$(cd "$(dirname "$0")/../.." && pwd)
ROOT="${GAMEPAD_VLA_WORKSPACE:-$HOME/gamepad_vla}"
DATA="${GAMEPAD_VLA_DATA:-$HOME/gamepad_vla_data}"
SPEC="${1:-configs/workcell/station_g20_v4_4_raised_interim.json}"
PORT="${PORT:-8780}"
[ -f "$CODE/$SPEC" ] || { echo "spec not found: $SPEC" >&2; exit 1; }

if state=$(curl -sf "http://127.0.0.1:$PORT/api/v1/status" 2>/dev/null); then
  rec=$(printf '%s' "$state" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("recording_status",{}).get("state","idle"))')
  if [ "$rec" != idle ]; then
    echo "recording state is '$rec'; stop the episode first" >&2; exit 1
  fi
fi
current="$ROOT/logs/collection-current"
if [ -f "$current" ] && [ -f "$(cat "$current")/pid" ]; then
  pid=$(cat "$(cat "$current")/pid")
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid"; for _ in $(seq 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
  fi
fi

O="$DATA/collect-$(date +%Y%m%d-%H%M%S)"; mkdir -p "$O"; echo "$O" > "$current"
cd "$CODE"
setsid nohup bash scripts/station/start_collection_server.sh --host 127.0.0.1 --port "$PORT" \
  --source ros2 --spec "$SPEC" --scene-variant gamepad-contact --arm-startup none \
  --output-root "$O/sessions" > "$O/server.log" 2>&1 < /dev/null &
echo $! > "$O/pid"
for _ in $(seq 60); do curl -sf "http://127.0.0.1:$PORT/api/v1/status" >/dev/null 2>&1 && break; sleep 0.5; done
curl -sf "http://127.0.0.1:$PORT/api/v1/status" >/dev/null && echo "collection server up on :$PORT with $SPEC (output $O)" \
  || { echo "server did not come up; see $O/server.log" >&2; exit 1; }
