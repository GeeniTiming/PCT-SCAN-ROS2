#!/usr/bin/env bash
# Launch the combined PCT + SCAN Garage demo owned by PCT-SCAN-ROS2.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
PROJECT_ROOT="$PCT_SCAN_PROJECT_ROOT"
TOMOGRAM="$PROJECT_ROOT/assets/garage/tomogram/garage_pct_0p10.pickle"
PCD_MAP="$PROJECT_ROOT/assets/garage/pcd/garage_scan_0p20.pcd"

for required_path in \
  "$TOMOGRAM" \
  "$PCD_MAP"
do
  if [[ ! -e "$required_path" ]]; then
    echo "[pct-scan-garage] Missing required path: $required_path" >&2
    exit 2
  fi
done

exec ros2 launch pct_scan_bridge garage_pct_scan.launch.py \
  tomogram_path:="$TOMOGRAM" \
  pcd_map_file:="$PCD_MAP" \
  "$@"
