#!/usr/bin/env bash
# Build the Isaac Sim 5.1.0 + Isaac Lab 2.3.2 environment on the compute node.
# Versions match the LinkerHand station (see references/linkerhand_station.md):
# Python 3.11, torch 2.7.0+cu128, rsl-rl-lib 3.1.2. Only the RSL-RL framework is
# installed. Downloads go direct (no proxy); the Isaac Lab tarball is fetched
# beforehand because GitHub needs the fallback proxy from this node.
#
# Usage: setup_isaaclab_env.sh <IsaacLab-v2.3.2.tar.gz>
set -euo pipefail

source "$(dirname "$0")/site_settings.sh"
W=$GAMEPAD_VLA_WORKSPACE
ENV=$W/envs/isaaclab-2.3.2
LAB=$W/external/IsaacLab-2.3.2
TARBALL=${1:?usage: setup_isaaclab_env.sh <IsaacLab-v2.3.2.tar.gz>}
CONDA=$GAMEPAD_VLA_CONDA

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export PIP_CACHE_DIR=$W/cache/pip XDG_CACHE_HOME=$W/cache TMPDIR=$W/tmp
export PIP_INDEX_URL=https://mirror.sjtu.edu.cn/pypi/web/simple
export PIP_EXTRA_INDEX_URL=https://pypi.nvidia.com
TORCH_INDEX=https://mirror.sjtu.edu.cn/pytorch-wheels/cu128

for path in "$ENV" "$LAB"; do
    if [[ -e $path ]]; then
        echo "$path already exists; refusing to overwrite" >&2
        exit 1
    fi
done

mkdir -p "$W/external"
tar -xzf "$TARBALL" -C "$W/external"
test -x "$LAB/isaaclab.sh"

"$CONDA" create -y -p "$ENV" python=3.11
PY=$ENV/bin/python
"$PY" -m pip install -U pip
"$PY" -m pip install torch==2.7.0 torchvision==0.22.0 --index-url "$TORCH_INDEX"
"$PY" -m pip install "isaacsim[all,extscache]==5.1.0"
# flatdict 4.0.1 (an isaaclab dependency) imports pkg_resources in setup.py, which
# setuptools>=81 removed; build it without isolation against an older setuptools.
"$PY" -m pip install "setuptools<81"
"$PY" -m pip install --no-build-isolation flatdict==4.0.1

# isaaclab.sh picks the interpreter from CONDA_PREFIX.
# The EULA acceptance was authorized by the user on 2026-10-14 (logs/decisions.log).
export CONDA_PREFIX=$ENV PATH=$ENV/bin:$PATH OMNI_KIT_ACCEPT_EULA=YES
cd "$LAB"
./isaaclab.sh --install rsl_rl
# isaaclab_mimic pulls ipython, which upgrades psutil; isaacsim-kernel pins 5.9.8.
"$PY" -m pip install psutil==5.9.8

"$PY" - <<'EOF'
import importlib.metadata as m
for name in ("torch", "torchvision", "isaacsim", "isaaclab", "isaaclab_rl", "rsl-rl-lib"):
    print(name, m.version(name))
EOF
echo "setup complete: $ENV"
