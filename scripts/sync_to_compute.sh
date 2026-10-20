#!/usr/bin/env bash
# Mirror this repository's working tree to the compute node for GPU runs.
# The node copy is disposable: edit here, sync, then run there.
set -euo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
source "$REPO/scripts/site_settings.sh"
DEST=$GAMEPAD_VLA_REMOTE_CODE
PROFILE=${GAMEPAD_VLA_COMPUTE_PROFILE:?Set compute.ssh_profile in the site configuration}
WITH_SITE=0
if [[ ${1:-} == --with-site-config ]]; then WITH_SITE=1; shift; fi
[[ $# == 0 ]] || { echo 'usage: sync_to_compute.sh [--with-site-config]' >&2; exit 2; }

PYTHON=${PYTHON:-python3}
ARCHIVE=$(mktemp --suffix=.tar.gz)
trap 'rm -f "$ARCHIVE"' EXIT
"$PYTHON" "$REPO/scripts/build_source_archive.py" --output "$ARCHIVE"
QUOTED_DEST=$("$PYTHON" -c 'import shlex,sys; print(shlex.quote(sys.argv[1]))' "$DEST")
# Keep the previous code snapshot; restore it if installing the new one fails.
"$PROFILE" --wohint "set -e
dest=$QUOTED_DEST
mkdir -p \"\$(dirname \"\$dest\")\"
stage=\$(mktemp -d \"\$dest.new.XXXXXX\")
trap 'rm -rf \"\$stage\"' EXIT
tar -xzf - -C \"\$stage\"
if [ -f \"\$dest/configs/site.local.json\" ]; then
    install -m 600 \"\$dest/configs/site.local.json\" \"\$stage/configs/site.local.json\"
fi
if [ -e \"\$dest\" ]; then
    backup=\$(mktemp -d \"\$dest.previous.XXXXXX\")
    mv \"\$dest\" \"\$backup/code\"
fi
if ! mv \"\$stage\" \"\$dest\"; then
    if [ -n \"\${backup:-}\" ]; then mv \"\$backup/code\" \"\$dest\"; fi
    exit 1
fi" < "$ARCHIVE"
if [[ $WITH_SITE == 1 ]]; then
    SITE_FILE=${GAMEPAD_VLA_SITE_CONFIG:-$REPO/configs/site.local.json}
    [[ -f $SITE_FILE ]] || { echo 'local site configuration is missing' >&2; exit 1; }
    "$PROFILE" --wohint "set -e; umask 077; dest=$QUOTED_DEST
        temporary=\$(mktemp \"\$dest/configs/site.local.XXXXXX\")
        cat > \"\$temporary\"
        mv \"\$temporary\" \"\$dest/configs/site.local.json\"" < "$SITE_FILE"
fi
echo "synced source to configured compute node"
