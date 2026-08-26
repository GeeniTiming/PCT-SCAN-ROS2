#!/usr/bin/env bash
# Shared runtime environment. Source this file; do not execute it directly.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This helper must be sourced by a PCT-SCAN-ROS2 launch script." >&2
  exit 2
fi

PCT_SCAN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PCT_SCAN_PROJECT_ROOT="$(cd "$PCT_SCAN_SCRIPT_DIR/.." && pwd -P)"
PCT_ROS_SETUP="/opt/ros/foxy/setup.bash"

if [[ ! -f "$PCT_ROS_SETUP" ]]; then
  echo "[pct-scan] ROS 2 Foxy was not found at /opt/ros/foxy." >&2
  return 2
fi
if [[ ! -f "$PCT_SCAN_PROJECT_ROOT/install/setup.bash" ]]; then
  echo "[pct-scan] Workspace is not built. Run ./scripts/build_workspace.sh." >&2
  return 2
fi

# Generated ROS setup files legitimately inspect optional unset variables.
set +u
source "$PCT_ROS_SETUP"
if [[ "${ROS_DISTRO:-}" != "foxy" ]]; then
  echo "[pct-scan] Expected ROS 2 Foxy, found ROS_DISTRO=${ROS_DISTRO:-unset}." >&2
  return 2
fi
if ! "$PCT_SCAN_SCRIPT_DIR/verify_m20pro_baseline.sh" --runtime; then
  return 2
fi

PCT_DEPS_ROOT="${PCT_DEPS_ROOT:-$PCT_SCAN_PROJECT_ROOT/.deps/install}"
if [[ ! -f "$PCT_DEPS_ROOT/gtsam-4.2/lib/libgtsam.so.4" ]]; then
  echo "[pct-scan] PCT dependencies are missing. Run ./scripts/setup_pct_dependencies.sh." >&2
  return 2
fi

export LD_LIBRARY_PATH="$PCT_DEPS_ROOT/gtsam-4.2/lib:$PCT_DEPS_ROOT/osqp-1.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source "$PCT_SCAN_PROJECT_ROOT/install/setup.bash"
set -u

export PCT_SCAN_ROS2_ROOT="$PCT_SCAN_PROJECT_ROOT"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PCT_SCAN_PROJECT_ROOT/.ros/log}"
mkdir -p "$ROS_LOG_DIR"
