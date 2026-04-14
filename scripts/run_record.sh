#!/bin/bash

set -euo pipefail

ROOT="/home/syhlabtop/workspace/openarm_lerobot"
CONFIG_DIR="$ROOT/configs"
TMP_DIR="$ROOT/.tmp"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_record.sh <preset> <run_name> [episode_count] [episode_time_s] [reset_time_s] [lerobot overrides...]

Optional environment overrides:
  OPENARM_RECORD_REPO_ID=<namespace>/<dataset_name>
  OPENARM_RECORD_PUSH_TO_HUB=1

Common LeRobot overrides passed after the wrapper args:
  --dataset.single_task="natural language task"
  --dataset.fps=30
  --robot.left_arm_config.cameras.left_wrist.fps=30
  --robot.left_arm_config.cameras.chest.fps=30
  --robot.right_arm_config.cameras.right_wrist.fps=30

Presets:
  nocam  - no cameras, no video
  rgb    - 3 cameras, RGB only, display off
  full   - 3 cameras, wrist depth on, display off

Examples:
  ./scripts/run_record.sh nocam test01
  ./scripts/run_record.sh rgb test02 2 20 20
  ./scripts/run_record.sh full test03 1 10 10
  OPENARM_RECORD_REPO_ID=KETI-IRRC/openarm_phase1_test12 OPENARM_RECORD_PUSH_TO_HUB=1 ./scripts/run_record.sh rgb test12
  ./scripts/run_record.sh rgb langtest 2 20 20 --dataset.fps=30 --dataset.single_task="Hand over the red cube"
  ./scripts/run_record.sh rgb stable20 2 20 20 --dataset.fps=20 --robot.left_arm_config.cameras.left_wrist.fps=20 --robot.left_arm_config.cameras.chest.fps=20 --robot.right_arm_config.cameras.right_wrist.fps=20
  OPENARM_RECORD_REPO_ID=syh4661/openarm_dualmanip_test03 OPENARM_RECORD_PUSH_TO_HUB=1 ./scripts/run_record.sh rgb openarm_dualmanip_test03 2 20 20 --dataset.fps=20 --dataset.single_task="pick the soda and put into the box" --robot.left_arm_config.cameras.left_wrist.fps=20 --robot.left_arm_config.cameras.chest.fps=20 --robot.right_arm_config.cameras.right_wrist.fps=20
EOF
}

PRESET="${1:-}"
RUN_NAME="${2:-}"
EPISODES="${3:-2}"
EPISODE_TIME="${4:-20}"
RESET_TIME="${5:-20}"
EXTRA_ARGS=("${@:6}")

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

TEMPLATE="${OPENARM_RECORD_TEMPLATE_OVERRIDE:-$CONFIG_DIR/record_${PRESET}.json}"
GENERATED="$TMP_DIR/record_${PRESET}_${RUN_NAME}.json"
DATA_ROOT="$ROOT/data/${RUN_NAME}"
REPO_ID="${OPENARM_RECORD_REPO_ID:-local/${RUN_NAME}}"
PUSH_TO_HUB="${OPENARM_RECORD_PUSH_TO_HUB:-0}"

case "$PUSH_TO_HUB" in
  0|1|true|false|True|False) ;;
  *)
    echo "[ERROR] OPENARM_RECORD_PUSH_TO_HUB must be 0/1/true/false, got: $PUSH_TO_HUB" >&2
    exit 1
    ;;
esac

mkdir -p "$TMP_DIR"

python3 - "$TEMPLATE" "$GENERATED" "$REPO_ID" "$DATA_ROOT" "$EPISODES" "$EPISODE_TIME" "$RESET_TIME" "$PUSH_TO_HUB" <<'PY'
import json
import sys
from pathlib import Path

template = Path(sys.argv[1])
generated = Path(sys.argv[2])
push_to_hub_raw = sys.argv[8].lower()

cfg = json.loads(template.read_text())
cfg["dataset"]["repo_id"] = sys.argv[3]
cfg["dataset"]["root"] = sys.argv[4]
cfg["dataset"]["num_episodes"] = int(sys.argv[5])
cfg["dataset"]["episode_time_s"] = int(sys.argv[6])
cfg["dataset"]["reset_time_s"] = int(sys.argv[7])
cfg["dataset"]["push_to_hub"] = push_to_hub_raw in {"1", "true"}

generated.write_text(json.dumps(cfg, indent=2))
print(generated)
PY

if [ "${OPENARM_RECORD_COMPAT_ONLY:-0}" = "1" ]; then
  echo "[INFO] Compatibility mode: generated config only"
  exit 0
fi

source "$ROOT/.venv312/bin/activate"
source "$ROOT/scripts/env_rsusb_py312.sh"

echo "[INFO] Preset      : $PRESET"
echo "[INFO] Run name    : $RUN_NAME"
echo "[INFO] Config path : $GENERATED"
echo "[INFO] Data root   : $DATA_ROOT"
echo "[INFO] Dataset ID  : $REPO_ID"
echo "[INFO] Push to Hub : $PUSH_TO_HUB"
if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
  echo "[INFO] Extra args  : ${EXTRA_ARGS[*]}"
fi

python -m lerobot.scripts.lerobot_record --config_path "$GENERATED" "${EXTRA_ARGS[@]}"
