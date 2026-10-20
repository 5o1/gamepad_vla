#!/usr/bin/env bash
# Source private JSON settings without treating configuration values as shell code.
SITE_REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SITE_EXPORTS=$(PYTHONPATH="$SITE_REPO/src${PYTHONPATH:+:$PYTHONPATH}" "${PYTHON:-python3}" -m gamepad_vla.site --shell) || return 1
eval "$SITE_EXPORTS"
unset SITE_EXPORTS
