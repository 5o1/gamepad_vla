#!/usr/bin/env bash
# Readback only. LeRobot 0.4.0 declares CODEBASE_VERSION=v3.0 in datasets/lerobot_dataset.py.
# Source: https://github.com/huggingface/lerobot/blob/v0.4.0/src/lerobot/datasets/lerobot_dataset.py
set -euo pipefail
source "$(dirname "$0")/site_settings.sh"
CHECK_ENV=$GAMEPAD_VLA_WORKSPACE/envs/lerobot-check-py310
BASE_PY=$GAMEPAD_VLA_WORKSPACE/envs/data-tools-py310/bin/python
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export PIP_CACHE_DIR=$GAMEPAD_VLA_WORKSPACE/cache/pip
export TMPDIR=$GAMEPAD_VLA_WORKSPACE/tmp
export HF_HOME=$GAMEPAD_VLA_WORKSPACE/cache/huggingface
export XDG_CACHE_HOME=$GAMEPAD_VLA_WORKSPACE/cache
mkdir -p "$PIP_CACHE_DIR" "$TMPDIR" "$HF_HOME"
PIP_ROUTE=()
if [[ ${1:-} == --proxy ]]; then
    # Only retry with this explicit flag after a recorded direct failure.
    PIP_ROUTE=(--proxy "${2:?supply proxy URL after a recorded direct failure}")
    shift 2
fi
[[ $# == 0 ]] || { echo 'usage: setup_lerobot_check_env.sh [--proxy URL]' >&2; exit 2; }
if [[ ! -e $CHECK_ENV ]]; then "$BASE_PY" -m venv "$CHECK_ENV"; fi
PY=$CHECK_ENV/bin/python
"$PY" -c 'import sys; assert sys.version_info[:2] == (3,10), sys.version'
CONSTRAINT=$(mktemp "$TMPDIR/lerobot-check.XXXXXX")
trap 'rm -f "$CONSTRAINT"' EXIT
# CPU wheels avoid installing a CUDA stack merely for numeric readback.
cat > "$CONSTRAINT" <<'EOF'
torch==2.7.1+cpu
torchvision==0.22.1+cpu
torchcodec==0.5
EOF
"$PY" -m pip install --disable-pip-version-check --retries 1 --timeout 30 \
    --index-url https://mirror.sjtu.edu.cn/pypi/web/simple \
    --extra-index-url https://download.pytorch.org/whl/cpu "${PIP_ROUTE[@]}" \
    -c "$CONSTRAINT" lerobot==0.4.0
"$PY" -m pip check
"$PY" -m pip freeze > "$CHECK_ENV/requirements-installed.txt"
HF_HUB_OFFLINE=1 "$PY" -c 'from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION; assert CODEBASE_VERSION == "v3.0"; print(CODEBASE_VERSION)'
