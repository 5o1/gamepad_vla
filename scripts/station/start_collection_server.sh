#!/usr/bin/env bash
# Collection simulation service only. Does not start any hardware input node.
set -euo pipefail
STATION_COLLECTION_ROOT="${GAMEPAD_VLA_WORKSPACE:-$HOME/gamepad_vla}"
STATION_COLLECTION_CODE=$(cd "$(dirname "$0")/../.." && pwd)
export GAMEPAD_VLA_WORKSPACE="$STATION_COLLECTION_ROOT"
export GAMEPAD_VLA_SITE_CONFIG="${GAMEPAD_VLA_SITE_CONFIG:-$STATION_COLLECTION_CODE/.local/station.json}"
export PYTHONPATH="$STATION_COLLECTION_CODE/src${PYTHONPATH:+:$PYTHONPATH}"
export XDG_CACHE_HOME="$STATION_COLLECTION_ROOT/cache" TMPDIR="$STATION_COLLECTION_ROOT/tmp"
export ROS_LOG_DIR="$STATION_COLLECTION_ROOT/logs/ros" ROS_HOME="$STATION_COLLECTION_ROOT/cache/ros"
export XBOX_GAMEPAD_PHYSICS_RESOURCES="$STATION_COLLECTION_ROOT/assets/xbox_gamepad_physics_v0.4.1/resources"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}" ROS_LOCALHOST_ONLY=1
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME" "$ROS_LOG_DIR" "$ROS_HOME"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$STATION_COLLECTION_CODE"
exec "$STATION_COLLECTION_ROOT/envs/sapien-3.0.3-py310/bin/python" scripts/collection_server.py "$@"
