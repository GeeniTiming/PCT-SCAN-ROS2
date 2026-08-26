#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
VENV_ROOT="${PCT_TOMOGRAPHY_VENV:-$PROJECT_ROOT/.venv-tomography}"

python3 -m venv "$VENV_ROOT"
"$VENV_ROOT/bin/python" -m pip install --upgrade pip wheel
"$VENV_ROOT/bin/python" -m pip install \
  "numpy==1.24.4" "scipy==1.10.1"

echo "[pct-scan] Python 3.8-compatible CPU tomography environment ready: $VENV_ROOT"
