#!/usr/bin/env bash
# Headless bottom-to-top PCT A* smoke test for the Garage tomogram.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/_pct_scan_env.sh"
PROJECT_ROOT="$PCT_SCAN_PROJECT_ROOT"
TOMOGRAM="$PROJECT_ROOT/assets/garage/tomogram/garage_pct_0p10.pickle"
ROUTE_OUTPUT="$PROJECT_ROOT/assets/garage/routes/f1_to_f5.csv"

for required_path in \
  "$TOMOGRAM"
do
  if [[ ! -e "$required_path" ]]; then
    echo "[garage-astar] Missing required path: $required_path" >&2
    exit 2
  fi
done

exec ros2 run pct_scan_bridge pct_route_smoke \
  --tomogram "$TOMOGRAM" \
  --output "$ROUTE_OUTPUT" \
  --require-cross-floor \
  --min-height-gain 10.0 \
  "$@"
