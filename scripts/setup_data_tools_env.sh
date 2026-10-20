#!/usr/bin/env bash
# Isolated Parquet conversion tools, authorized by reviewer ruling 1.
# Run on the compute node after scripts/sync_to_compute.sh. No simulation env is edited.
set -euo pipefail

source "$(dirname "$0")/site_settings.sh"
W=$GAMEPAD_VLA_WORKSPACE
DATA_ENV=$W/envs/data-tools-py310
BASE_PY=$W/envs/sapien-3.0.3-py310/bin/python
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export PIP_CACHE_DIR=$W/cache/pip XDG_CACHE_HOME=$W/cache TMPDIR=$W/tmp
export PIP_INDEX_URL=https://mirror.sjtu.edu.cn/pypi/web/simple
mkdir -p "$W/cache/pip" "$W/tmp"

if [[ ! -e $DATA_ENV ]]; then
    "$BASE_PY" -m venv "$DATA_ENV"
fi
PY=$DATA_ENV/bin/python
"$PY" -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version'
# Direct mirror only. A failed download stops here, preserving evidence before
# an explicitly selected proxy retry; the script never silently switches routes.
"$PY" -m pip install --disable-pip-version-check --retries 1 --timeout 30 pyarrow==25.0.1 numpy==2.2.6
"$PY" - <<'PY'
import sys
import importlib.metadata as metadata
print(sys.version)
for name in ("pyarrow", "numpy"):
    print(name, metadata.version(name))
PY
echo "setup complete: $DATA_ENV"
