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

## Immediate Next Steps

1. Install Python 3.12
2. Install `ffmpeg`
3. Install `uv` or create equivalent isolated environment
4. Install LeRobot in editable mode from `../lerobot`
5. Implement OpenArm recorder adapter
