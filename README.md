# openarm_lerobot

Thin integration layer between OpenArm teleoperation/control and LeRobot-compatible data collection and training workflows.

## Goals

- Keep real-time robot control in existing OpenArm repos.
- Add a recorder/export layer for LeRobot-compatible datasets.
- Add camera integration after mounts are ready.
- Establish practical OpenArm baselines before larger VLA work.

## Repository Scope

- `docs/` — setup notes, operator flow, dataset schema
- `configs/` — robot, recorder, camera, and baseline configs
- `src/openarm_lerobot/` — adapter and recorder code
- `tests/` — smoke tests and schema validation

## Phase 1

1. Base LeRobot platform present locally in sibling repo `../lerobot`
2. OpenArm integration repo scaffolded here
3. No-camera recorder path defined first
4. Camera support added after chest/wrist mounts are ready
5. Baselines start with proprio-only BC, then single-camera ACT

## Current Decisions

- OpenArm teleoperation remains in `../openarm_teleop`
- ROS/runtime remains in existing OpenArm repos
- LeRobot is used as a sibling dependency, not vendored into OpenArm repos
- One chest camera + two wrist cameras are planned hardware, but not required for initial repo bring-up

## Data Acquisition / Recording

`scripts/run_record.sh` is the thin entrypoint for recording. It selects the LeRobot recording preset and generates the recorder config, so this README only captures the operator view.

### Presets

Available presets at a high level:

- `nocam` for proprio-only recording
- `rgb` for the three-camera RGB-only setup
- `full` for the three-camera phase 1 setup with wrist depth enabled

### Default behavior

By default, the wrapper now targets **30 FPS** in the preset configs, records to a local dataset ID like `local/<run_name>`, and keeps Hub upload disabled. Operators can still override the generated config at runtime without editing JSON by hand.

### Control surface

The control surface is split into three layers:

- wrapper positional args: `preset`, `run_name`, `episode_count`, `episode_time_s`, `reset_time_s`
- wrapper environment overrides: Hub dataset ID and upload toggle
- raw LeRobot CLI overrides appended after the wrapper args for dataset metadata and camera tuning

### Uploading to Hugging Face

If you want the recording to target a real Hugging Face dataset such as `KETI-IRRC/openarm_phase1_test12`, run it with explicit overrides:

```bash
OPENARM_RECORD_REPO_ID=KETI-IRRC/openarm_phase1_test12 \
OPENARM_RECORD_PUSH_TO_HUB=1 \
./scripts/run_record.sh rgb test12 2 20 20
```

This keeps the local copy under `data/<run_name>` while setting `dataset.repo_id` to the Hub dataset ID and enabling upload.

### Runtime dataset overrides

You can also append normal LeRobot CLI overrides after the wrapper arguments. Those are forwarded to `lerobot_record` at runtime, which is useful for fields such as `dataset.fps`, `dataset.single_task`, or other VLA-oriented metadata:

```bash
./scripts/run_record.sh rgb langtest 2 20 20 \
  --dataset.fps=30 \
  --dataset.single_task="Hand over the red cube to the other arm"
```

Useful runtime fields include:

- `--dataset.single_task="natural language task"`
- `--dataset.fps=<hz>`
- `--dataset.private=true|false`
- `--robot.left_arm_config.cameras.left_wrist.fps=<hz>`
- `--robot.left_arm_config.cameras.chest.fps=<hz>`
- `--robot.right_arm_config.cameras.right_wrist.fps=<hz>`

Per-camera FPS is controlled separately from `dataset.fps`, so operators can tune the image streams directly when hardware cannot sustain the default 30 FPS:

```bash
./scripts/run_record.sh rgb stable20 2 20 20 \
  --dataset.fps=20 \
  --robot.left_arm_config.cameras.left_wrist.fps=20 \
  --robot.left_arm_config.cameras.chest.fps=20 \
  --robot.right_arm_config.cameras.right_wrist.fps=20
```

### FPS tuning guide

Recommended tuning order:

1. start with the default `30 / 30` (dataset + cameras)
2. if multi-camera stability is poor, try `20 / 20`
3. if the system still drops frames or glitches, fall back to `15 / 15`

The fixed wrapper-generated values still come from the preset and the first five arguments, but extra overrides are applied on top when LeRobot loads the config, so teleoperators or data collectors can handle task language, FPS, camera rates, and Hub upload behavior directly from the command line.

### Operational note

`--dataset.fps` changes the record loop and dataset sampling target. Camera stream rates stay under each `--robot.*.cameras.*.fps` field, so raising dataset FPS alone does not automatically raise camera FPS.

Before recording, make sure you have a Python 3.12 environment, the LeRobot checkout available locally, and the RSUSB / RealSense runtime in place for any camera preset.

For the exact command shape and flags, see `docs/lerobot_openarm_record_command.md`. For the camera runtime and setup context, see `docs/realsense_rsusb_runbook.md` and `docs/next_steps.md`.

## Immediate Next Steps

1. Install Python 3.12
2. Install `ffmpeg`
3. Install `uv` or create equivalent isolated environment
4. Install LeRobot in editable mode from `../lerobot`
5. Implement OpenArm recorder adapter
