#!/usr/bin/env bash
# 采集台一键录制：中间相机 + 双臂状态 + 双手状态 → ros2 bag（供仿真孪生标定与关节约定核对）。
#
# 只启动状态类节点：rokae_state_publisher_node（读关节角/末端位姿，不发运动指令）、
# LinkerHand SDK（读手状态）、RealSense 相机。不启动任何遥操作节点，不发任何运动指令。
# 注意（2026-10-16 实测）：rokae_state_publisher_node 连接时会给两条臂【上电】（日志
# "Robot powered on successfully"），并把工具坐标系设为零、设置默认速度/转弯区；结束本脚本
# 不会自动下电，臂会保持上电并停在原姿态。手 SDK 启动时是否有初始化动作未经核实。
# 运行时请人在旁边看着。
# 结束（正常、Ctrl+C 或出错）时自动调用 ~/gamepad_vla_estop/estop.sh 清理全部相关进程。
#
# 用法：bash ~/gamepad_vla_estop/twin_calib_record.sh [录制秒数，默认 10]
# 可选环境变量：LEFT_CAN=can1 RIGHT_CAN=can0 HAND_JOINT=G20 NO_HAND=1（不启动手 SDK）
set -uo pipefail
source "$(dirname "$0")/../site_settings.sh" || exit 1
for key in STATION_WORKSPACE STATION_ESTOP_SCRIPT STATION_DATA_ROOT STATION_LEFT_ARM_IP STATION_RIGHT_ARM_IP STATION_LEFT_LOCAL_IP STATION_RIGHT_LOCAL_IP; do
  [ -n "${!key}" ] || { echo "Missing site setting: $key" >&2; exit 1; }
done

DURATION="${1:-10}"
LEFT_CAN="${LEFT_CAN:-$STATION_LEFT_CAN}"
RIGHT_CAN="${RIGHT_CAN:-$STATION_RIGHT_CAN}"
HAND_JOINT="${HAND_JOINT:-$STATION_HAND_JOINT}"
NO_HAND="${NO_HAND:-0}"
WS=$STATION_WORKSPACE
ESTOP=$STATION_ESTOP_SCRIPT
TS=$(date +%Y%m%d_%H%M%S)
OUT=$STATION_DATA_ROOT/bags/twin_calib_${TS}  # keep our data out of the station workstation tree
LOGS="${OUT}_logs"
PIDS=()

say() { echo -e "\033[1;36m[twin_calib]\033[0m $*"; }
die() { echo -e "\033[1;31m[twin_calib] Error:\033[0m $*" >&2; exit 1; }

cleanup() {
  trap - EXIT INT TERM
  say "Cleanup: running the stop script to end all related processes…"
  bash "$ESTOP" || true
  [ -d "$OUT" ] && say "bag: $OUT" && (set +u; source "$STATION_ROS_SETUP"; ros2 bag info "$OUT" 2>/dev/null | head -25)
  say "Logs: $LOGS"
}
trap cleanup EXIT
trap 'say "Interrupted"; exit 130' INT TERM

ros_env() {
  # ROS/conda setup scripts reference unset variables, so nounset is off while sourcing them.
  set +u
  if [ -f "$STATION_CONDA_SETUP" ]; then
    source "$STATION_CONDA_SETUP"
    conda deactivate 2>/dev/null; conda deactivate 2>/dev/null
  fi
  source "$STATION_ROS_SETUP"
  source "$WS/ros2_ws/install/setup.bash"
  set -u
}

wait_topic() {  # wait_topic <topic> <seconds>
  local t=$1 n=$2
  for _ in $(seq 1 "$n"); do ros2 topic list 2>/dev/null | grep -qx "$t" && return 0; sleep 1; done
  return 1
}

# ---------- 预检查 ----------
[ -x "$ESTOP" ] || [ -f "$ESTOP" ] || die "Stop script not found: $ESTOP"
mkdir -p "$LOGS"
say "Preflight checks…"
CAMS=$(for d in /sys/bus/usb/devices/*; do [ "$(cat $d/idVendor 2>/dev/null)" = 8086 ] && cat $d/product; done | grep -ci realsense)
[ "$CAMS" = 1 ] || die "Found $CAMS RealSense cameras on USB (expected 1; keep only the camera at the top of the column)"
for ip in "$STATION_RIGHT_ARM_IP" "$STATION_LEFT_ARM_IP"; do ping -c1 -W1 $ip >/dev/null || die "Arm controller $ip is unreachable"; done
if [ "$NO_HAND" != 1 ]; then
  for c in "$LEFT_CAN" "$RIGHT_CAN"; do ip -br link show "$c" 2>/dev/null | grep -q UP || die "$c is not UP"; done
fi
CONFLICT=$(pgrep -af 'impedance_teleop|rt_teleop|teleop_bridge|handretarget|linkerta|RobotControl_node|test_luoshi' | grep -v pgrep)
[ -z "$CONFLICT" ] || die "Teleoperation or control processes are running. Stop them first:\n$CONFLICT"

cat <<EOF

Will start: center camera (if stopped), dual-arm state publisher (30 Hz), $( [ "$NO_HAND" = 1 ] && echo "(hand SDK disabled)" || echo "dual-hand SDK ($LEFT_CAN/$RIGHT_CAN, $HAND_JOINT)" )
Then record for ${DURATION} seconds to: $OUT
No teleoperation or motion commands. The arm state node POWERS ON both arms; they stay powered after this script ends. Use RobotHMI to power off.
Confirm that someone is watching the arms and hands, and both hands are visible in the center camera.
Software stop: run bash $ESTOP in another terminal

EOF
read -r -p "Type yes to start: " ANS
[ "$ANS" = yes ] || die "Cancelled"

ros_env

# ---------- 相机 ----------
if ros2 topic list 2>/dev/null | grep -qx /camera_3/camera/color/image_raw; then
  say "Center camera already running. Reusing it."
else
  say "Starting center camera…"
  ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera_3 camera_name:=camera \
    rgb_camera.color_profile:=640,480,30 enable_depth:=true depth_module.depth_profile:=640,480,30 \
    align_depth.enable:=true pointcloud.enable:=false >"$LOGS/cam.log" 2>&1 &
  PIDS+=($!)
  wait_topic /camera_3/camera/color/image_raw 20 || die "Camera topic not found. See $LOGS/cam.log"
fi
grep -a -o "Device Serial No: [0-9]*" "$LOGS/cam.log" 2>/dev/null | head -1 | tee "$LOGS/camera_serial.txt"

# ---------- 臂状态 ----------
say "Starting dual-arm state publisher (no motion commands)…"
# xCoreSDK writes ./_rokae_log_/ in the working directory; ~/_rokae_log_ is root-owned, so run from $LOGS.
(cd "$LOGS" && exec ros2 run robot_control rokae_state_publisher_node --ros-args \
  -p right_ip:="$STATION_RIGHT_ARM_IP" -p left_ip:="$STATION_LEFT_ARM_IP" \
  -p right_local_ip:="$STATION_RIGHT_LOCAL_IP" -p left_local_ip:="$STATION_LEFT_LOCAL_IP" \
  -p publish_rate_hz:=30.0) >"$LOGS/arm.log" 2>&1 &
PIDS+=($!)
wait_topic /rokae/left/joint_state 20 || die "Arm state topic not found. See $LOGS/arm.log"

# ---------- 手 SDK ----------
HAND_TOPICS=()
if [ "$NO_HAND" != 1 ]; then
  say "Starting dual-hand SDK… (watch the hands)"
  ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py \
    left_can:="$LEFT_CAN" right_can:="$RIGHT_CAN" hand_joint:="$HAND_JOINT" >"$LOGS/hand.log" 2>&1 &
  PIDS+=($!)
  wait_topic /cb_left_hand_state 20 || die "Hand state topic not found. See $LOGS/hand.log"
  HAND_TOPICS=(/cb_left_hand_state /cb_right_hand_state)
fi

# ---------- 话题检查 ----------
say "Checking topic rates (3 seconds each)…"
for t in /camera_3/camera/color/image_raw /rokae/left/joint_state /rokae/left/ee_pose; do
  HZ=$(timeout 4 ros2 topic hz "$t" 2>/dev/null)
  RATE=$(printf '%s\n' "$HZ" | grep 'average rate' | tail -1)
  echo "  $t: ${RATE:-No data}"
done | tee "$LOGS/topic_rates.txt"

# ---------- 录制 ----------
say "Recording for ${DURATION} seconds…"
# --foreground: without it the recorder runs in a separate process group and is stopped (SIGTTOU) on tty access.
timeout --foreground -s INT "$((DURATION + 2))" ros2 bag record -o "$OUT" \
  /camera_3/camera/color/image_raw /camera_3/camera/color/camera_info \
  /camera_3/camera/aligned_depth_to_color/image_raw /camera_3/camera/aligned_depth_to_color/camera_info \
  /camera_3/camera/extrinsics/depth_to_color \
  /rokae/left/joint_state /rokae/right/joint_state /rokae/left/ee_pose /rokae/right/ee_pose \
  "${HAND_TOPICS[@]}" >"$LOGS/bag.log" 2>&1
say "Recording complete. Both arms are still powered on. Use RobotHMI to power off."
