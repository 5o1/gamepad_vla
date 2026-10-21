#!/usr/bin/env bash
# Local software stop for simulation collection/input only; never contacts a robot.
# This does not cut motor power. Do not use it as a physical emergency stop.
set -euo pipefail
pkill -INT -f '(^|[ /])(run_collection\.py|forward_collection\.py)( |$)' || true
pkill -TERM -f '(^|[ /])(manus_data_publisher|handretarget|linkerta)( |$)' || true
echo 'Simulation collection and input processes received stop signals; check their logs.'
