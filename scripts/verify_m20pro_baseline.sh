#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
MODE="${1:---static}"
EXPECTED_CMAKE_MAX="3.16"
failures=0

fail() {
  echo "[m20pro] FAIL: $*" >&2
  failures=$((failures + 1))
}

version_is_greater() {
  local candidate="$1"
  local baseline="$2"
  [[ "$candidate" != "$baseline" ]] && \
    [[ "$(printf '%s\n%s\n' "$candidate" "$baseline" | sort -V | tail -n 1)" == "$candidate" ]]
}

check_cmake_files() {
  local cmake_file required_version
  while IFS= read -r -d '' cmake_file; do
    required_version="$(sed -nE 's/^[[:space:]]*cmake_minimum_required\([[:space:]]*VERSION[[:space:]]+([0-9]+\.[0-9]+).*/\1/p' "$cmake_file" | head -n 1)"
    if [[ -n "$required_version" ]] && version_is_greater "$required_version" "$EXPECTED_CMAKE_MAX"; then
      fail "${cmake_file#$PROJECT_ROOT/} requires CMake $required_version; M20Pro baseline is 3.16.3"
    fi
  done < <(find "$PROJECT_ROOT/src" -name CMakeLists.txt -print0)
}

check_static_baseline() {
  check_cmake_files
  [[ -f "$PROJECT_ROOT/src/simulator/COLCON_IGNORE" ]] || \
    fail "src/simulator/COLCON_IGNORE is missing; board builds would include simulator dependencies"
  grep -q 'cmake_minimum_required(VERSION 3.16)' \
    "$PROJECT_ROOT/patches/osqp-v1.0.0-cmake-3.16.patch" || \
    fail "OSQP CMake 3.16 compatibility patch is missing"
  grep -q 'PYBIND11_VERSION="2.10.4"' \
    "$PROJECT_ROOT/scripts/setup_pct_dependencies.sh" || \
    fail "private pybind11 2.10.4 dependency pin is missing"
  if grep -R -n 'pybind11_vendor' "$PROJECT_ROOT/src/pct" \
      --include=CMakeLists.txt --include=package.xml >/dev/null; then
    fail "active PCT packages still depend on ROS pybind11_vendor"
  fi
  if grep -R -nE --exclude=verify_m20pro_baseline.sh \
      '/opt/ros/humble|ros-humble-' "$PROJECT_ROOT/scripts" >/dev/null; then
    fail "runtime scripts contain a ROS 2 Humble path or package"
  fi
}

check_runtime_baseline() {
  local os_id="" os_version="" cmake_version python_version compiler_version
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    os_id="${ID:-}"
    os_version="${VERSION_ID:-}"
  fi
  [[ "$os_id" == "ubuntu" && "$os_version" == "20.04" ]] || \
    fail "expected Ubuntu 20.04, found ${os_id:-unknown} ${os_version:-unknown}"
  [[ "$(uname -m)" == "aarch64" ]] || fail "expected aarch64, found $(uname -m)"

  command -v cmake >/dev/null || fail "cmake is not installed"
  if command -v cmake >/dev/null; then
    cmake_version="$(cmake --version | awk 'NR == 1 { print $3 }')"
    [[ "$cmake_version" == 3.16.* ]] || \
      fail "expected Ubuntu 20.04 system CMake 3.16.x, found $cmake_version"
  fi

  command -v python3 >/dev/null || fail "python3 is not installed"
  if command -v python3 >/dev/null; then
    python_version="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    [[ "$python_version" == "3.8" ]] || fail "expected Python 3.8, found $python_version"
  fi

  command -v c++ >/dev/null || fail "the system C++ compiler is not installed"
  if command -v c++ >/dev/null; then
    compiler_version="$(c++ -dumpfullversion -dumpversion)"
    [[ "$compiler_version" == 9.* ]] || fail "expected GCC 9, found $compiler_version"
  fi

  [[ -f /opt/ros/foxy/setup.bash ]] || fail "/opt/ros/foxy/setup.bash is missing"
  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "foxy" ]]; then
    fail "expected ROS_DISTRO=foxy, found ROS_DISTRO=$ROS_DISTRO"
  fi

  if command -v dpkg-query >/dev/null && \
      dpkg-query -W -f='${db:Status-Status}\n' libgoogle-glog-dev 2>/dev/null | grep -qx installed; then
    fail "libgoogle-glog-dev is installed; Lightning-LM requires its pinned glog 0.6.0"
  fi
}

case "$MODE" in
  --static)
    check_static_baseline
    ;;
  --runtime)
    check_static_baseline
    check_runtime_baseline
    ;;
  *)
    echo "Usage: $0 [--static|--runtime]" >&2
    exit 2
    ;;
esac

if ((failures > 0)); then
  echo "[m20pro] $failures compatibility check(s) failed." >&2
  exit 1
fi

echo "[m20pro] $MODE compatibility checks passed."
