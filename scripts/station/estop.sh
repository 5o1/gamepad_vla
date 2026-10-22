#!/usr/bin/env bash
# gamepad_vla 软件急停：结束所有可能向机械臂/灵巧手发指令或占用设备的进程。
# 这不是硬件急停：它依赖本机正常运行；无法切断电机电源。
# 用法：bash ~/gamepad_vla_estop/estop.sh
PATTERN='rokae_[a-z_]*node|RobotControl_node|test_luoshi|linker_hand_ros2_sdk|linker_hand_double|handretarget|linkerta|manus_data_publisher|realsense2_camera|rs_launch|ros2 bag record'
echo "[estop] $(date '+%F %T') Starting"
tmux kill-session -t twin_calib 2>/dev/null && echo "[estop] Stopped tmux session twin_calib"
pkill -INT -f 'ros2 bag record' 2>/dev/null && echo "[estop] Sent SIGINT to ros2 bag record (save bag)"
pkill -TERM -f "$PATTERN" 2>/dev/null
sleep 1.5
pkill -KILL -f "$PATTERN" 2>/dev/null
LEFT=$(pgrep -af "$PATTERN" | grep -v -E 'estop\.sh|pgrep')
if [ -n "$LEFT" ]; then echo "[estop] Still running:"; echo "$LEFT"; exit 1; fi
echo "[estop] Done: no arm, hand, camera, or recording processes remain."
echo "[estop] To power off the arms, use RobotHMI or the control cabinet power switch."
