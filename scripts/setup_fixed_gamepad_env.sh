#!/usr/bin/env bash
# Isolated Python 3.11/MuJoCo contact-training environment; run on the compute node.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
source "$REPO/scripts/site_settings.sh"
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
export PIP_CACHE_DIR="$GAMEPAD_VLA_WORKSPACE/cache/pip" XDG_CACHE_HOME="$GAMEPAD_VLA_WORKSPACE/cache" TMPDIR="$GAMEPAD_VLA_WORKSPACE/tmp"
export PIP_INDEX_URL=${PIP_INDEX_URL:-https://mirror.sjtu.edu.cn/pypi/web/simple}
mkdir -p "$PIP_CACHE_DIR" "$TMPDIR"
BOOTSTRAP_PYTHON=
SOURCE=
WHEEL=
INHERIT=()
while [[ $# -gt 0 ]]; do
    case $1 in
        --python) BOOTSTRAP_PYTHON=${2:?}; shift 2 ;;
        --gamepad-source) SOURCE=${2:?}; shift 2 ;;
        --mujoco-wheel) WHEEL=${2:?}; shift 2 ;;
        --inherit-site-packages) INHERIT=(--system-site-packages); shift ;;
        *) echo 'usage: setup_fixed_gamepad_env.sh --python python3.11 --gamepad-source CHECKOUT --mujoco-wheel WHEEL [--inherit-site-packages]' >&2; exit 2 ;;
    esac
done
[[ -n $BOOTSTRAP_PYTHON && -n $SOURCE && -n $WHEEL ]] || { echo 'Python, source checkout and patched wheel are required' >&2; exit 2; }
SOURCE=$(realpath "$SOURCE")
WHEEL=$(realpath "$WHEEL")
ENV=$GAMEPAD_VLA_WORKSPACE/envs/fixed-gamepad-mujoco-py311
RESOURCES=$GAMEPAD_VLA_WORKSPACE/assets/xbox_gamepad_physics_mujoco_84f43e6/resources
[[ ! -e $ENV && ! -e $RESOURCES ]] || { echo 'environment or resources already exist; select a new workspace' >&2; exit 1; }
"$BOOTSTRAP_PYTHON" - "$REPO" "$SOURCE" "$WHEEL" <<'PY'
import hashlib,json,subprocess,sys,zipfile
from pathlib import Path
assert sys.version_info[:2] == (3,11),sys.version
manifest=json.loads((Path(sys.argv[1])/'references/fixed_gamepad_resources.json').read_text())
revision=subprocess.check_output(['git','-C',sys.argv[2],'rev-parse','HEAD'],text=True).strip()
if revision != manifest['gamepad']['revision']:
    raise SystemExit('gamepad checkout differs from the pinned task revision')
if subprocess.check_output(['git','-C',sys.argv[2],'status','--porcelain','--untracked-files=no']):
    raise SystemExit('gamepad source has tracked modifications')
digest=hashlib.sha256(Path(sys.argv[3]).read_bytes()).hexdigest()
if digest != manifest['mujoco']['sha256']:
    # A rebuild from the documented source has a different binary hash. Its
    # declared source identity must still match; this is not contact validation.
    with zipfile.ZipFile(sys.argv[3]) as archive:
        identity=json.loads(archive.read('mujoco/CONCON_BUILD.json'))
    if (identity.get('source_commit') != manifest['mujoco']['revision']
            or identity.get('distribution_version') != manifest['mujoco']['distribution']
            or identity.get('variant') != 'patched'):
        raise SystemExit('MuJoCo rebuild source identity differs from the pinned dependency')
print(json.dumps({'mujoco_wheel_sha256':digest,'reviewed_binary':digest==manifest['mujoco']['sha256']}))
PY
"$BOOTSTRAP_PYTHON" -m venv "${INHERIT[@]}" "$ENV"
PY=$ENV/bin/python
"$PY" -m pip install -c "$REPO/constraints/fixed-gamepad-py311.txt" \
    "$WHEEL" "$SOURCE[build]" -e "$REPO[train,dev]"
"$PY" -m xbox_gamepad_physics.builders.build_resources --output "$RESOURCES" \
    --inputs "$SOURCE" --backend mujoco \
    --calibration "$SOURCE/src/xbox_gamepad_physics/build_data/calibration/mujoco_m2.json"
"$PY" -m pip check
"$PY" -m pip freeze > "$ENV/requirements-installed.txt"
echo "ready: $ENV"
