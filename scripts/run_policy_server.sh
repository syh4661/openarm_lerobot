#!/bin/bash
# run_policy_server.sh — Run OpenPI Pi0.5 policy server on GPU machine
#
# Usage:
#   ./run_policy_server.sh [port]
#
# Requirements:
#   - NVIDIA GPU with >= 8GB VRAM (RTX 3090 / A100 recommended)
#   - CUDA 12.x, Python 3.11+
#   - uv (https://docs.astral.sh/uv/)

set -euo pipefail

PORT="${1:-8000}"
CHECKPOINT_DIR="${2:-/home/syhlabtop/workspace/openarm_pi05_finetuned}"

echo "=== OpenArm Pi0.5 Policy Server ==="
echo "Port:            $PORT"
echo "Checkpoint dir:  $CHECKPOINT_DIR"
echo ""

# Clone openpi fork if not present
if [ ! -d "openpi_saurabh" ]; then
    echo "[INFO] Cloning openpi fork..."
    git clone https://github.com/AiSaurabhPatil/openpi.git openpi_saurabh
    cd openpi_saurabh
    git submodule update --init --recursive
else
    cd openpi_saurabh
fi

# Install dependencies
echo "[INFO] Installing dependencies..."
GIT_LFS_SKIP_SMUDGE=1 uv sync

# Download checkpoint if not present
if [ ! -f "$CHECKPOINT_DIR/params/manifest.ocdbt" ]; then
    echo "[INFO] Downloading checkpoint (12GB, may take a while)..."
    mkdir -p "$CHECKPOINT_DIR"
    uv run python -c "
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id='AiSaurabhPatil/openarm-pi05-finetuned',
    local_dir='$CHECKPOINT_DIR',
)
"
fi

# Run policy server
echo "[INFO] Starting policy server on port $PORT..."
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.85

uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi05_openarm \
    --policy.dir="$CHECKPOINT_DIR" \
    --port="$PORT"
