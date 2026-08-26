#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
PCT_DEPS_ROOT="${PCT_DEPS_ROOT:-$PROJECT_ROOT/.deps/install}"
PCT_ROS_DISTRO="${PCT_ROS_DISTRO:-${ROS_DISTRO:-foxy}}"
ROS_SETUP="/opt/ros/$PCT_ROS_DISTRO/setup.bash"

if [[ ! -f "$PCT_DEPS_ROOT/gtsam-4.2/lib/cmake/GTSAM/GTSAMConfig.cmake" ]]; then
  echo "[pct-scan] Missing GTSAM. Run ./scripts/setup_pct_dependencies.sh first." >&2
  exit 2
fi
if [[ ! -d "$PCT_DEPS_ROOT/osqp-1.0.0/lib/cmake/osqp" ]]; then
  echo "[pct-scan] Missing OSQP. Run ./scripts/setup_pct_dependencies.sh first." >&2
  exit 2
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[pct-scan] ROS 2 setup not found: $ROS_SETUP" >&2
  echo "[pct-scan] Set PCT_ROS_DISTRO when using a distribution other than Foxy." >&2
  exit 2
fi

set +u
source "$ROS_SETUP"
set -u
export GTSAM_DIR="$PCT_DEPS_ROOT/gtsam-4.2/lib/cmake/GTSAM"
export osqp_DIR="$PCT_DEPS_ROOT/osqp-1.0.0/lib/cmake/osqp"
export OSQP_DIR="$osqp_DIR"
export CMAKE_PREFIX_PATH="$GTSAM_DIR:$osqp_DIR${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PCT_DEPS_ROOT/gtsam-4.2/lib:$PCT_DEPS_ROOT/osqp-1.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$PROJECT_ROOT"
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DPCT_TARGET_ARCH="$(uname -m)" \
    -DGTSAM_DIR="$GTSAM_DIR" \
    -Dosqp_DIR="$osqp_DIR" \
  "$@"
