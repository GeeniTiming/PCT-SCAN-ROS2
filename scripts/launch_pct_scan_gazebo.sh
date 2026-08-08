#!/usr/bin/env bash
# Launch PCT + SCAN together with the visual-only Garage Gazebo scene.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

exec "$SCRIPT_DIR/launch_pct_scan_garage.sh" \
  use_gazebo:=true \
  "$@"
