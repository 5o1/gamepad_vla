#!/usr/bin/env bash
# Build the SAPIEN 3.0.3 + xbox_gamepad_physics v0.4.1 environment on the compute node
# for the collection workstation's first simulation backend (RoboTwin SDF gamepad).
# Python 3.10 and SAPIEN 3.0.3 follow the gamepad release's documented target.
# Downloads go direct (no proxy) through the SJTU mirror; the gamepad wheel and
# resource zip come from the pinned release copy in assets/.
#
# Usage: setup_sapien_env.sh [--python /path/to/python3.10]
set -euo pipefail

source "$(dirname "$0")/site_settings.sh"
W=$GAMEPAD_VLA_WORKSPACE
REPO=$(cd "$(dirname "$0")/.." && pwd)
ENV=$W/envs/sapien-3.0.3-py310
RELEASE=$W/assets/xbox_gamepad_physics_v0.4.1/release
RESOURCES=$W/assets/xbox_gamepad_physics_v0.4.1/resources
CONDA=$GAMEPAD_VLA_CONDA
BOOTSTRAP_PYTHON=
if [[ ${1:-} == --python ]]; then BOOTSTRAP_PYTHON=${2:?missing Python path}; shift 2; fi
[[ $# == 0 ]] || { echo 'usage: setup_sapien_env.sh [--python python3.10]' >&2; exit 2; }

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export PIP_CACHE_DIR=$W/cache/pip XDG_CACHE_HOME=$W/cache TMPDIR=$W/tmp
export PIP_INDEX_URL=${PIP_INDEX_URL:-https://mirror.sjtu.edu.cn/pypi/web/simple}
mkdir -p "$W/cache/pip" "$W/tmp"

for path in "$ENV" "$RESOURCES"; do
    if [[ -e $path ]]; then
        echo "$path already exists; refusing to overwrite" >&2
        exit 1
    fi
done
(cd "$RELEASE" && sha256sum -c SHA256SUMS)

if [[ -n $BOOTSTRAP_PYTHON ]]; then
    "$BOOTSTRAP_PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version'
    "$BOOTSTRAP_PYTHON" -m venv "$ENV"
else
    "$CONDA" create -y -p "$ENV" python=3.10
fi
PY=$ENV/bin/python
"$PY" -m pip install -U pip
"$PY" -m pip install -c "$REPO/constraints/sapien-py310.txt" \
    "$RELEASE/xbox_gamepad_physics-0.4.1-py3-none-any.whl" -e "$REPO[sim,web,dev]"
"$PY" -m xbox_gamepad_physics build-resources \
    --archive "$RELEASE/xbox_gamepad_physics-resources-0.4.1.zip" \
    --resources "$RESOURCES" --offline

"$PY" - <<'EOF'
import importlib.metadata as m
for name in ("sapien", "xbox_gamepad_physics", "numpy", "trimesh"):
    print(name, m.version(name))
EOF
echo "setup complete: $ENV (resources: $RESOURCES)"
