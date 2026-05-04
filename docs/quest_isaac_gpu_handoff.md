# Quest Isaac GPU Handoff

## Current status

This repo is the tracked source of truth for the next agent. `.sisyphus/` plans, evidence, and notepads are ignored after clone, so this document is the handoff that survives a fresh checkout.

Completed pushed commits:

- `4528747` `docs(quest): freeze isaac real control contract`
- `7d7bd5a` `test(quest): validate spatial replay fixtures`
- `aa9f014` `test(quest): gate replay through processor ik path`
- `77f830b` `test(isaac): inspect local openarm environment`
- `2cffd6b` `test(quest): validate dry run recording path`

Current local machine status:

- Isaac Lab is blocked on this machine, `scripts/inspect_openarm_isaac_env.py` reports `status: fail` with `reason: isaac_unavailable`.
- Real hardware is not ready on this machine, and it must stay blocked until the Isaac gates are frozen on the larger GPU PC.
- Current warning notes from the run: Placo neutral URDF self collision warnings, SocketCAN was not properly shut down after a failed CAN connect, and no permission issue was observed.

## Required environment on the GPU PC

- clone this repo
- sibling `../lerobot/src`
- sibling `../openarm_description`
- Isaac Lab and the OpenArm Isaac env on the GPU PC
- Python 3.12 runtime, `.venv312` or equivalent
- `ROS_PACKAGE_PATH=/path/to/workspace`

## Fresh clone verification

Run these from the repo root on the GPU PC after activating the Python 3.12 / Isaac env:

```bash
source .venv312/bin/activate
export PYTHONPATH="$PWD/src:/path/to/workspace/lerobot/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_PACKAGE_PATH="/path/to/workspace${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
python3 -m py_compile src/openarm_lerobot/*.py scripts/*.py
python3 scripts/validate_quest_spatial_replay.py --trace configs/quest_spatial_replay/right_axis_smoke.json --report .sisyphus/evidence/task-2-right-schema.json
python3 scripts/test_quest_replay_processor_gate.py --trace configs/quest_spatial_replay/right_axis_smoke.json --report .sisyphus/evidence/task-3-replay-processor-right.json
python3 scripts/inspect_openarm_isaac_env.py --report .sisyphus/evidence/task-4-isaac-env.json
```

## Next implementation tasks on the GPU PC

- implement `scripts/validate_quest_isaac_replay.py`
- implement `scripts/validate_quest_isaac_bimanual_replay.py`
- freeze evidence after both pass, or fail them explicitly with the real reason

Task numbering used here:

- Task 2: existing replay schema validator (`scripts/validate_quest_spatial_replay.py`)
- Task 3: existing replay processor / IK gate (`scripts/test_quest_replay_processor_gate.py`)
- Task 4: Isaac env inspection (`scripts/inspect_openarm_isaac_env.py`)
- Task 5/6: next GPU work for Isaac replay, then bimanual replay

## Pass and fail criteria

### Task 4, Isaac env inspection

Pass when the report says `status: pass`, `reason: pass`, and the env is registered with discovered Isaac metadata, including `joint_names_discovered: true` and `tcp_frame_discovered: true`.

Fail when Isaac Lab is missing, OpenArm Isaac registration is missing, the env is unregistered, or discovered joint or TCP metadata cannot be read from Isaac.

### Task 5, unimanual Isaac replay

Pass when the Isaac replay gate runs on the GPU PC and the frozen thresholds are met:

- `max_tracking_error_m <= 0.03`
- `control_rate_hz >= 25`
- `collision_count == 0`
- `nan_count == 0`
- `disabled_drift_m <= 0.005`
- `stop_latency_s <= 0.5`

Fail when any threshold is missed, the Isaac env is missing, or the replay cannot be run honestly.

### Task 6, bimanual Isaac replay

Pass only when bimanual replay is explicit, both arms are covered, and the same frozen thresholds are met for the bimanual trace.

Fail when only one arm is covered, the bimanual trace is incomplete, the env is missing, or any threshold is missed.

## Transition rule to real hardware

Do not run safe shutdown, real motion, or recording gates until Task 5 unimanual passes and Task 6 bimanual status is explicit.

## Notes for the next operator

- The frozen Quest contract lives in `docs/quest_isaac_real_control_contract.md`.
- `scripts/inspect_openarm_isaac_env.py` is the current Task 4 evidence source.
- `scripts/validate_quest_recording_dry_run.py` is the current safe failure path for recording dry runs.
- `scripts/test_quest_replay_processor_gate.py` is the current replay and IK gate reference.
