#!/bin/bash

set -euo pipefail

ROOT="/home/syhlabtop/workspace/openarm_lerobot"
LEROBOT_SRC="/home/syhlabtop/workspace/lerobot"
VENV_DIR="$ROOT/.venv312"

/usr/bin/python3.12 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

python -m pip install --upgrade pip setuptools wheel

python -m pip install -e "$LEROBOT_SRC"

echo "[INFO] LeRobot py312 bootstrap complete"
echo "[INFO] Activate with: source $VENV_DIR/bin/activate"
