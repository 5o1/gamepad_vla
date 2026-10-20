#!/usr/bin/env bash
# Software stop only; it cannot remove motor power. Hardware authorization is required.
set -euo pipefail
source "$(dirname "$0")/site_settings.sh"
: "${STATION_SSH_PROFILE:?Set station.ssh_profile in the site configuration}"
: "${STATION_ESTOP_SCRIPT:?Set station.estop_script in the site configuration}"
printf -v command 'bash %q' "$STATION_ESTOP_SCRIPT"
exec "$STATION_SSH_PROFILE" -n "$command"
