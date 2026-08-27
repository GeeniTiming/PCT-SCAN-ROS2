#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
DEPS_ROOT="${PCT_DEPS_ROOT:-$PROJECT_ROOT/.deps/install}"
SOURCE_ROOT="$PROJECT_ROOT/.deps/src"
BUILD_ROOT="$PROJECT_ROOT/.deps/build"
JOBS="${PCT_BUILD_JOBS:-2}"
TARGET_ARCH="$(uname -m)"
CMAKE_COMMAND="${PCT_CMAKE_COMMAND:-cmake}"

GTSAM_VERSION="4.2"
GTSAM_COMMIT="4f66a491ffc83cf092d0d818b11dc35135521612"
OSQP_VERSION="1.0.0"
OSQP_COMMIT="236713ce9a56c182ac3230d52108f952afce1523"
OSQP_PATCH="$PROJECT_ROOT/patches/osqp-v1.0.0-cmake-3.16.patch"
PYBIND11_VERSION="2.10.4"
PYBIND11_COMMIT="5b0a6fc2017fcc176545afe3e09c9f9885283242"
MINIMUM_CMAKE_VERSION="3.16"

command -v git >/dev/null
command -v "$CMAKE_COMMAND" >/dev/null
command -v ninja >/dev/null
mkdir -p "$DEPS_ROOT" "$SOURCE_ROOT" "$BUILD_ROOT"

cmake_version="$($CMAKE_COMMAND --version | awk 'NR == 1 { print $3 }')"
if [[ "$(printf '%s\n%s\n' "$MINIMUM_CMAKE_VERSION" "$cmake_version" | sort -V | head -n 1)" != "$MINIMUM_CMAKE_VERSION" ]]; then
  echo "[pct-scan] CMake $MINIMUM_CMAKE_VERSION or newer is required; found $cmake_version." >&2
  echo "[pct-scan] Ubuntu 20.04's system CMake 3.16.3 is supported and should not be replaced." >&2
  exit 2
fi

baseline_mode="--runtime"
if [[ "${PCT_ALLOW_NON_M20_HOST:-0}" == "1" ]]; then
  baseline_mode="--static"
fi
"$SCRIPT_DIR/verify_m20pro_baseline.sh" "$baseline_mode"

clone_if_missing() {
  local source_dir="$1"
  shift
  if [[ ! -d "$source_dir/.git" ]]; then
    git clone --recursive --depth 1 "$@" "$source_dir"
  fi
}

verify_revision() {
  local source_dir="$1"
  local dependency_name="$2"
  local expected_commit="$3"
  local actual_commit
  actual_commit="$(git -C "$source_dir" rev-parse HEAD)"
  if [[ "$actual_commit" != "$expected_commit" ]]; then
    echo "[pct-scan] $dependency_name source revision mismatch." >&2
    echo "[pct-scan] Expected $expected_commit, found $actual_commit in $source_dir." >&2
    echo "[pct-scan] Move that generated source directory aside and run this script again." >&2
    exit 2
  fi
}

apply_patch_once() {
  local source_dir="$1"
  local patch_file="$2"
  if git -C "$source_dir" apply --check "$patch_file"; then
    git -C "$source_dir" apply "$patch_file"
  elif git -C "$source_dir" apply --reverse --check "$patch_file"; then
    echo "[pct-scan] Compatibility patch already applied: $(basename "$patch_file")"
  else
    echo "[pct-scan] Compatibility patch does not match the pinned source: $patch_file" >&2
    exit 2
  fi
}

clone_if_missing "$SOURCE_ROOT/gtsam-$GTSAM_VERSION" \
  --branch "$GTSAM_VERSION" https://github.com/borglab/gtsam.git
clone_if_missing "$SOURCE_ROOT/osqp-$OSQP_VERSION" \
  --branch "v$OSQP_VERSION" https://github.com/osqp/osqp.git
clone_if_missing "$SOURCE_ROOT/pybind11-$PYBIND11_VERSION" \
  --branch "v$PYBIND11_VERSION" https://github.com/pybind/pybind11.git

verify_revision "$SOURCE_ROOT/gtsam-$GTSAM_VERSION" "GTSAM $GTSAM_VERSION" "$GTSAM_COMMIT"
verify_revision "$SOURCE_ROOT/osqp-$OSQP_VERSION" "OSQP $OSQP_VERSION" "$OSQP_COMMIT"
verify_revision "$SOURCE_ROOT/pybind11-$PYBIND11_VERSION" "pybind11 $PYBIND11_VERSION" "$PYBIND11_COMMIT"
apply_patch_once "$SOURCE_ROOT/osqp-$OSQP_VERSION" "$OSQP_PATCH"

"$CMAKE_COMMAND" -S "$SOURCE_ROOT/pybind11-$PYBIND11_VERSION" -B "$BUILD_ROOT/pybind11-$PYBIND11_VERSION" -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT/pybind11-$PYBIND11_VERSION" \
  -DPYBIND11_INSTALL=ON \
  -DPYBIND11_TEST=OFF
"$CMAKE_COMMAND" --build "$BUILD_ROOT/pybind11-$PYBIND11_VERSION" --parallel "$JOBS"
"$CMAKE_COMMAND" --install "$BUILD_ROOT/pybind11-$PYBIND11_VERSION"

"$CMAKE_COMMAND" -S "$SOURCE_ROOT/gtsam-$GTSAM_VERSION" -B "$BUILD_ROOT/gtsam-$GTSAM_VERSION" -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT/gtsam-$GTSAM_VERSION" \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
"$CMAKE_COMMAND" --build "$BUILD_ROOT/gtsam-$GTSAM_VERSION" --parallel "$JOBS"
"$CMAKE_COMMAND" --install "$BUILD_ROOT/gtsam-$GTSAM_VERSION"

"$CMAKE_COMMAND" -S "$SOURCE_ROOT/osqp-$OSQP_VERSION" -B "$BUILD_ROOT/osqp-$OSQP_VERSION" -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT/osqp-$OSQP_VERSION" \
  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
  -DOSQP_ALGEBRA_BACKEND=builtin \
  -DOSQP_BUILD_DEMO_EXE=OFF \
  -DOSQP_BUILD_UNITTESTS=OFF \
  -DOSQP_CODEGEN=OFF \
  -DOSQP_ENABLE_DERIVATIVES=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
"$CMAKE_COMMAND" --build "$BUILD_ROOT/osqp-$OSQP_VERSION" --parallel "$JOBS"
"$CMAKE_COMMAND" --install "$BUILD_ROOT/osqp-$OSQP_VERSION"

echo "[pct-scan] Dependencies for $TARGET_ARCH installed under $DEPS_ROOT"
