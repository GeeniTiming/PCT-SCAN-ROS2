#!/usr/bin/env bash
# Launch the PCT-SCAN-ROS2 Garage global-planner bridge.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
PROJECT_ROOT="$PCT_SCAN_PROJECT_ROOT"
TOMOGRAM="$PROJECT_ROOT/assets/garage/tomogram/garage_pct_0p10.pickle"

for required_path in \
  "$TOMOGRAM"
do
  if [[ ! -e "$required_path" ]]; then
    echo "[garage-bridge] Missing required path: $required_path" >&2
    exit 2
  fi
done

exec ros2 launch pct_scan_bridge garage_bridge.launch.py \
  tomogram_path:="$TOMOGRAM" \
  rsg_root:="$PROJECT_ROOT/assets/garage" \
  "$@"
