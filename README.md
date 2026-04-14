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

Available presets at a high level:

- `nocam` for proprio-only recording
- `rgb` for a single-camera RGB setup
- `full` for the multi-camera phase 1 setup

Before recording, make sure you have a Python 3.12 environment, the LeRobot checkout available locally, and the RSUSB / RealSense runtime in place for any camera preset.

For the exact command shape and flags, see `docs/lerobot_openarm_record_command.md`. For the camera runtime and setup context, see `docs/realsense_rsusb_runbook.md` and `docs/next_steps.md`.

## Immediate Next Steps

1. Install Python 3.12
2. Install `ffmpeg`
3. Install `uv` or create equivalent isolated environment
4. Install LeRobot in editable mode from `../lerobot`
5. Implement OpenArm recorder adapter
