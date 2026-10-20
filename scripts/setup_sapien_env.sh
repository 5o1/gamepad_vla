#!/usr/bin/env bash
# Build the SAPIEN 3.0.3 + xbox_gamepad_physics v0.3.1 environment on the compute node
# for the collection workstation's first simulation backend (RoboTwin SDF gamepad).
# Python 3.10 and SAPIEN 3.0.3 follow the gamepad release's documented target.
# Downloads go direct (no proxy) through the SJTU mirror; the gamepad wheel and
# resource zip come from the pinned release copy in assets/.
#
# Usage: setup_sapien_env.sh
set -euo pipefail

source "$(dirname "$0")/site_settings.sh"
W=$GAMEPAD_VLA_WORKSPACE
REPO=$(cd "$(dirname "$0")/.." && pwd)
ENV=$W/envs/sapien-3.0.3-py310
RELEASE=$W/assets/xbox_gamepad_physics_v0.3.1/release
RESOURCES=$W/assets/xbox_gamepad_physics_v0.3.1/resources
CONDA=$GAMEPAD_VLA_CONDA

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export PIP_CACHE_DIR=$W/cache/pip XDG_CACHE_HOME=$W/cache TMPDIR=$W/tmp
export PIP_INDEX_URL=https://mirror.sjtu.edu.cn/pypi/web/simple
mkdir -p "$W/cache/pip" "$W/tmp"

for path in "$ENV" "$RESOURCES"; do
    if [[ -e $path ]]; then
        echo "$path already exists; refusing to overwrite" >&2
        exit 1
    fi
done
(cd "$RELEASE" && sha256sum -c SHA256SUMS)

"$CONDA" create -y -p "$ENV" python=3.10
PY=$ENV/bin/python
"$PY" -m pip install -U pip
"$PY" -m pip install -c "$REPO/constraints/sapien-py310.txt" \
    --find-links "$RELEASE" -e "$REPO[sim,web,dev]"
"$PY" -m xbox_gamepad_physics build-resources \
    --archive "$RELEASE/xbox_gamepad_physics-resources-0.3.1.zip" \
    --resources "$RESOURCES" --offline

"$PY" - <<'EOF'
import importlib.metadata as m
for name in ("sapien", "xbox_gamepad_physics", "numpy", "trimesh"):
    print(name, m.version(name))
EOF
echo "setup complete: $ENV (resources: $RESOURCES)"
