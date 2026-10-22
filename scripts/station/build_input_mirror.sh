#!/usr/bin/env bash
# Copy upstream input sources once, apply our config delta, and build in isolation.
set -euo pipefail
CODE=$(cd "$(dirname "$0")/../.." && pwd)
if [[ -z ${GAMEPAD_VLA_SITE_CONFIG:-} && -f "$CODE/.local/station.json" ]]; then
  export GAMEPAD_VLA_SITE_CONFIG="$CODE/.local/station.json"
fi
source "$CODE/scripts/site_settings.sh"
SOURCE=${STATION_INPUT_SOURCE:-$HOME/workstation}
MIRROR=${STATION_INPUT_MIRROR:-$HOME/gamepad_vla/station_ws}
ROS_SETUP=${STATION_ROS_SETUP:-/opt/ros/humble/setup.bash}
LINKERTA_ONLY=0
while (($#)); do
  case "$1" in
    --source) SOURCE=$2; shift 2 ;;
    --mirror) MIRROR=$2; shift 2 ;;
    --ros-setup) ROS_SETUP=$2; shift 2 ;;
    --linkerta-only) LINKERTA_ONLY=1; shift ;;
    -h|--help) echo "Usage: $0 [--source DIR] [--mirror DIR] [--ros-setup FILE] [--linkerta-only]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
SOURCE=$(realpath -m "${SOURCE/#\~/$HOME}")
MIRROR=$(realpath -m "${MIRROR/#\~/$HOME}")
ROS_SETUP=${ROS_SETUP/#\~/$HOME}
case "$MIRROR/" in "$SOURCE/"*) echo "Mirror must be outside the read-only source" >&2; exit 1 ;; esac
if (( ! LINKERTA_ONLY )); then
  [[ ! -e "$MIRROR/src" ]] || { echo "$MIRROR/src exists; use --linkerta-only to repair LinkerTA, or choose a new mirror" >&2; exit 1; }
  mkdir -p "$MIRROR/src/manus_ros2_bundle"
  cp -a "$SOURCE/linkerhand-tele-manus/retarget/src/linkerhand_retarget" "$MIRROR/src/"
  cp -a "$SOURCE/linkerhand-tele-arm-main/src/linkerta" "$MIRROR/src/"
  for package in manus_ros2 manus_ros2_msgs ManusSDK; do
    cp -a "$SOURCE/linkerhand-tele-manus/manus_ros2/$package" "$MIRROR/src/manus_ros2_bundle/"
  done
  find "$MIRROR/src" -type d -name __pycache__ -prune -exec rm -rf {} +
  python3 - "$MIRROR/src/linkerhand_retarget/linkerhand_retarget/config/base_config.yml" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
section = re.search(r"^manus_dex:\n(?:[ \t].*\n|\n)+", text, re.M)
if section is None:
    raise SystemExit("manus_dex configuration missing")
block, count = re.subn(r"(^  thumb_ergo:\n(?:    .*\n)*?    enabled:) (?:true|false)(?=\s*(?:#.*)?$)",
                       r"\1 false", section.group(), count=1, flags=re.M)
if count != 1:
    raise SystemExit("manus_dex.thumb_ergo.enabled missing")
path.write_text(text[:section.start()] + block + text[section.end():])
PY
  cat > "$MIRROR/MIRROR.md" <<EOF
# gamepad_vla station input-chain mirror

Copied on $(date +%F) from $SOURCE (read-only source):
- linkerhand-tele-manus/retarget/src/linkerhand_retarget -> src/linkerhand_retarget
- linkerhand-tele-arm-main/src/linkerta -> src/linkerta
- linkerhand-tele-manus/manus_ros2/{manus_ros2,manus_ros2_msgs,ManusSDK} -> src/manus_ros2_bundle/

Our only source delta: config/base_config.yml, manus_dex.thumb_ergo.enabled: false.
Upstream enabled angle_direct thumb opposition for a bottle-cap task. Our gamepad
task uses the vector retargeter's thumb solution; motion suitability needs onsite validation.
Build with PYTHONNOUSERSITE=1 and LinkerTA USE_ROS2=ON.
Compare when needed with diff -r <source-package> <mirror-package>; no checksum inventory.
EOF
fi
cd "$MIRROR"
set +u
source "$ROS_SETUP"
set -u
export PYTHONNOUSERSITE=1
if (( LINKERTA_ONLY )); then
  colcon build --symlink-install --packages-select linkerta --cmake-args -DUSE_ROS2=ON
else
  colcon build --symlink-install --packages-select manus_ros2_msgs manus_ros2 linkerhand_retarget
  colcon build --symlink-install --packages-select linkerta --cmake-args -DUSE_ROS2=ON
fi
set +u
source "$MIRROR/install/local_setup.bash"
set -u
ros2 pkg prefix linkerta
ros2 launch linkerta run.launch.py --show-args
