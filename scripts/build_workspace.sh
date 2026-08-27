#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
PCT_DEPS_ROOT="${PCT_DEPS_ROOT:-$PROJECT_ROOT/.deps/install}"
ROS_SETUP="/opt/ros/foxy/setup.bash"

baseline_mode="--runtime"
if [[ "${PCT_ALLOW_NON_M20_HOST:-0}" == "1" ]]; then
  baseline_mode="--static"
fi
"$SCRIPT_DIR/verify_m20pro_baseline.sh" "$baseline_mode"

if [[ ! -f "$PCT_DEPS_ROOT/gtsam-4.2/lib/cmake/GTSAM/GTSAMConfig.cmake" ]]; then
  echo "[pct-scan] Missing GTSAM. Run ./scripts/setup_pct_dependencies.sh first." >&2
  exit 2
fi
if [[ ! -d "$PCT_DEPS_ROOT/osqp-1.0.0/lib/cmake/osqp" ]]; then
  echo "[pct-scan] Missing OSQP. Run ./scripts/setup_pct_dependencies.sh first." >&2
  exit 2
fi
if [[ ! -f "$PCT_DEPS_ROOT/pybind11-2.10.4/share/cmake/pybind11/pybind11Config.cmake" ]]; then
  echo "[pct-scan] Missing private pybind11. Run ./scripts/setup_pct_dependencies.sh again." >&2
  exit 2
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[pct-scan] ROS 2 setup not found: $ROS_SETUP" >&2
  echo "[pct-scan] This deployment branch targets ROS 2 Foxy on Ubuntu 20.04." >&2
  exit 2
fi

set +u
source "$ROS_SETUP"
set -u
if [[ "${ROS_DISTRO:-}" != "foxy" ]]; then
  echo "[pct-scan] Expected ROS 2 Foxy, found ROS_DISTRO=${ROS_DISTRO:-unset}." >&2
  exit 2
fi
export CMAKE_PREFIX_PATH="$PCT_DEPS_ROOT/pybind11-2.10.4:$PCT_DEPS_ROOT/gtsam-4.2:$PCT_DEPS_ROOT/osqp-1.0.0${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PCT_DEPS_ROOT/gtsam-4.2/lib:$PCT_DEPS_ROOT/osqp-1.0.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$PROJECT_ROOT"
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  "$@"
