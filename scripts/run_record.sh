#!/bin/bash

set -euo pipefail

ROOT="/home/syhlabtop/workspace/openarm_lerobot"
CONFIG_DIR="$ROOT/configs"
TMP_DIR="$ROOT/.tmp"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_record.sh <preset> <run_name> [episode_count] [episode_time_s] [reset_time_s]

Presets:
  nocam  - no cameras, no video
  rgb    - 3 cameras, RGB only, display off
  full   - 3 cameras, wrist depth on, display off

Examples:
  ./scripts/run_record.sh nocam test01
  ./scripts/run_record.sh rgb test02 2 20 20
  ./scripts/run_record.sh full test03 1 10 10
EOF
}

PRESET="${1:-}"
RUN_NAME="${2:-}"
EPISODES="${3:-2}"
EPISODE_TIME="${4:-20}"
RESET_TIME="${5:-20}"

if [ -z "$PRESET" ] || [ -z "$RUN_NAME" ]; then
  usage
  exit 1
fi

case "$PRESET" in
  nocam|rgb|full) ;;
  *)
    echo "[ERROR] Unknown preset: $PRESET" >&2
    usage
    exit 1
    ;;
esac

TEMPLATE="$CONFIG_DIR/record_${PRESET}.json"
GENERATED="$TMP_DIR/record_${PRESET}_${RUN_NAME}.json"
DATA_ROOT="$ROOT/data/${RUN_NAME}"
REPO_ID="local/${RUN_NAME}"

mkdir -p "$TMP_DIR"

python3 - <<PY
import json
from pathlib import Path

template = Path("$TEMPLATE")
generated = Path("$GENERATED")

cfg = json.loads(template.read_text())
cfg["dataset"]["repo_id"] = "$REPO_ID"
cfg["dataset"]["root"] = "$DATA_ROOT"
cfg["dataset"]["num_episodes"] = int("$EPISODES")
cfg["dataset"]["episode_time_s"] = int("$EPISODE_TIME")
cfg["dataset"]["reset_time_s"] = int("$RESET_TIME")

generated.write_text(json.dumps(cfg, indent=2))
print(generated)
PY

source "$ROOT/.venv312/bin/activate"
source "$ROOT/scripts/env_rsusb_py312.sh"

echo "[INFO] Preset      : $PRESET"
echo "[INFO] Run name    : $RUN_NAME"
echo "[INFO] Config path : $GENERATED"
echo "[INFO] Data root   : $DATA_ROOT"

python -m lerobot.scripts.lerobot_record --config_path "$GENERATED"
