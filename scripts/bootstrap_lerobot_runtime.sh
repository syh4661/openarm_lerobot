#!/bin/bash

set -euo pipefail

ROOT="/home/syhlabtop/workspace/openarm_lerobot"
LEROBOT_SRC="/home/syhlabtop/workspace/lerobot"
VENV_DIR="$ROOT/.venv310"

python3 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

python -m pip install --upgrade pip setuptools wheel

# Minimal runtime first, before full editable install
python -m pip install \
  draccus==0.10.0 \
  pillow \
  opencv-python-headless \
  numpy

# Install LeRobot without dependency resolution first to allow staged validation.
python -m pip install --no-deps -e "$LEROBOT_SRC"

echo "[INFO] Bootstrap complete"
echo "[INFO] Activate with: source $VENV_DIR/bin/activate"
echo "[INFO] Then source RSUSB wrapper: source $ROOT/scripts/env_rsusb_py310.sh"
