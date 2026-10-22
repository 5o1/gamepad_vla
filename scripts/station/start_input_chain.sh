#!/usr/bin/env bash
# Authorized onsite input only: Manus, retarget, LinkerTA. No slave hand/arm nodes.
set -euo pipefail
CODE=$(cd "$(dirname "$0")/../.." && pwd)
if [[ -z ${GAMEPAD_VLA_SITE_CONFIG:-} && -f "$CODE/.local/station.json" ]]; then
  export GAMEPAD_VLA_SITE_CONFIG="$CODE/.local/station.json"
fi
source "$CODE/scripts/site_settings.sh"
MIRROR=${STATION_INPUT_MIRROR:-$HOME/gamepad_vla/station_ws}
LOG_ROOT=${STATION_INPUT_LOG_ROOT:-$HOME/gamepad_vla/logs}
RETARGET_PYTHON=${STATION_RETARGET_PYTHON:-$HOME/miniconda3/envs/workstation/bin/python}
ROS_SETUP=${STATION_ROS_SETUP:-/opt/ros/humble/setup.bash}
while (($#)); do
  case "$1" in
    --mirror) MIRROR=$2; shift 2 ;;
    --log-root) LOG_ROOT=$2; shift 2 ;;
    --retarget-python) RETARGET_PYTHON=$2; shift 2 ;;
    --ros-setup) ROS_SETUP=$2; shift 2 ;;
    -h|--help) echo "Usage: $0 [--mirror DIR] [--log-root DIR] [--retarget-python FILE] [--ros-setup FILE]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
MIRROR=$(realpath "${MIRROR/#\~/$HOME}")
LOG_ROOT=$(realpath -m "${LOG_ROOT/#\~/$HOME}")
RETARGET_PYTHON=${RETARGET_PYTHON/#\~/$HOME}
ROS_SETUP=${ROS_SETUP/#\~/$HOME}
if pgrep -af 'linker_hand_ros2_sdk|linker_hand_double|rokae_' | grep -vqE 'pgrep|cursorsandbox'; then
  echo "检测到手 SDK 或珞石节点在运行，拒绝启动" >&2; exit 1
fi
if pgrep -af 'manus_data_publisher|handretarget|linkerta_node' | grep -vqE 'pgrep|cursorsandbox'; then
  echo "输入节点仍在运行；先用 stop_input_chain.sh 停止已登记的输入链" >&2; exit 1
fi
# Clear inherited workspace overlays; local_setup loads only our mirror on Humble.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
set +u
source "$ROS_SETUP"
source "$MIRROR/install/local_setup.bash"
set -u
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
for package in manus_ros2 linkerhand_retarget linkerta; do
  prefix=$(ros2 pkg prefix "$package")
  [[ "$prefix" == "$MIRROR/install/$package" ]] || { echo "$package resolved outside mirror: $prefix" >&2; exit 1; }
  printf '%s: %s\n' "$package" "$prefix"
done
# Read the installed configuration actually used by run.launch.py, never upstream.
"$RETARGET_PYTHON" - "$MIRROR/install/linkerta/share/linkerta/config/lta.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as stream:
    calibration = yaml.safe_load(stream)["linkerta_node"]["ros__parameters"]["calibration"]
if type(calibration) is not int or calibration != 0:
    raise SystemExit("lta.yaml calibration 不是 0，拒绝启动")
print(f"calibration: {calibration} ({sys.argv[1]})")
PY
L="$LOG_ROOT/m2-$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$L/ros" "$LOG_ROOT/../cache/ros"
export ROS_LOG_DIR="$L/ros" ROS_HOME="$(realpath "$LOG_ROOT/../cache/ros")"
# The upstream node does not print this setting; record its source config in its log.
"$RETARGET_PYTHON" - "$MIRROR/src/linkerhand_retarget/linkerhand_retarget/config/base_config.yml" > "$L/retarget.log" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as stream:
    enabled = yaml.safe_load(stream)["manus_dex"]["thumb_ergo"]["enabled"]
print(f"Input config: manus_dex.thumb_ergo.enabled: {str(enabled).lower()} ({sys.argv[1]})")
PY
printf '%s\n' "$L" > "$LOG_ROOT/m2-current"
cd "$L"
setsid nohup ros2 run manus_ros2 manus_data_publisher > manus.log 2>&1 < /dev/null &
echo "manus $!" >> pids
sleep 3
# Explicit interpreter bypasses the installed entry point's /usr/bin/python3 shebang.
setsid nohup "$RETARGET_PYTHON" "$MIRROR/install/linkerhand_retarget/lib/linkerhand_retarget/handretarget" \
  --ros-args -p motion_type:=manus_dex -p enabled_hands:=left,right -p calibration:=false \
  >> retarget.log 2>&1 < /dev/null &
echo "retarget $!" >> pids
setsid nohup ros2 launch linkerta run.launch.py > linkerta.log 2>&1 < /dev/null &
echo "linkerta $!" >> pids
sleep 8
failed=0
while read -r name pid; do
  if ! kill -0 "$pid" 2>/dev/null; then echo "$name exited; see $L/$name.log" >&2; failed=1; fi
done < pids
cat pids
head -n 1 retarget.log
tail -n 5 retarget.log linkerta.log
echo "Input logs: $L (ROS domain 88, loopback only)"
exit "$failed"
