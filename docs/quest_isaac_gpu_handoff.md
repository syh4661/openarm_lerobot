# Quest Isaac GPU Handoff

## Current status

This repo is the tracked source of truth for the next agent. `.sisyphus/` plans, evidence, and notepads are ignored after clone, so this document is the handoff that survives a fresh checkout.

Current run update, 2026-05-07:

- Isaac Lab was found at `/home/syhai/IsaacLab/isaaclab.sh`. Running with `/home/syhai/IsaacLab/isaaclab.sh -p` imports `isaaclab`, while `omni.isaac.lab` fails with `ModuleNotFoundError`.
- Task 4 was rerun with the IsaacLab interpreter. The report is an honest fail, not an Isaac Lab install failure: `status: fail`, `reason: openarm_isaac_unavailable`, `availability.openarm_isaac: missing`, `joint_names_discovered: false`, and `tcp_frame_discovered: false`.
- Task 5 was skipped and not run because Task 4 failed with `reason: openarm_isaac_unavailable`; the metadata booleans stayed false.
- Task 6 was skipped and not run because Task 4 failed with `reason: openarm_isaac_unavailable`; bimanual Isaac status is not explicit.
- Real hardware remains blocked because OpenArm Isaac is unavailable, Task 5 unimanual has not passed, and Task 6 bimanual status is not explicit.
- No real hardware, CAN, recording, safe shutdown, `send_action`, or Hub upload is authorized by this handoff.

Current evidence paths:

- `.sisyphus/evidence/task-9-isaac-imports.log`: IsaacLab wrapper exists, `isaaclab` import succeeds, `omni.isaac.lab` import fails.
- `.sisyphus/evidence/task-4-isaac-env.json`: Task 4 machine-readable status and reason.
- `.sisyphus/evidence/task-4-isaac-env.log`: Task 4 stdout, stderr, and exit code capture.

Completed pushed commits:

- `4528747` `docs(quest): freeze isaac real control contract`
- `7d7bd5a` `test(quest): validate spatial replay fixtures`
- `aa9f014` `test(quest): gate replay through processor ik path`
- `77f830b` `test(isaac): inspect local openarm environment`
- `2cffd6b` `test(quest): validate dry run recording path`
- `a7a26da` `fix(init): defer safe_followers import to avoid eager can dependency`

Current local machine status:

- Isaac Lab itself is present through `/home/syhai/IsaacLab/isaaclab.sh -p`, but OpenArm Isaac is missing from that environment.
- `scripts/inspect_openarm_isaac_env.py` reports `status: fail` with `reason: openarm_isaac_unavailable`.
- Real hardware is not ready on this machine, and it must stay blocked until Task 5 unimanual passes and Task 6 bimanual status is explicit.
- Historical warning notes still matter for later hardware work: Placo neutral URDF self collision warnings, SocketCAN was not properly shut down after a failed CAN connect, and no permission issue was observed.

## Required environment on the GPU PC

- clone this repo
- sibling `../lerobot/src`
- sibling `../openarm_description`
- Isaac Lab and the local OpenArm Isaac package in the same active environment
- Python 3.12 runtime, `.venv312` or equivalent
- `ROS_PACKAGE_PATH=/path/to/workspace`
- current blocker to fix before Task 5 or Task 6: install or activate the local `openarm_isaac_lab` package so the OpenArm Isaac import succeeds

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

For the current installed IsaacLab path, Task 4 was run through `/home/syhai/IsaacLab/isaaclab.sh -p`. That interpreter is acceptable for `isaaclab`, but it still needs the OpenArm Isaac package before Task 4 can pass.

## Next implementation tasks on the GPU PC

- fix the active Isaac environment so the OpenArm Isaac package imports inside `/home/syhai/IsaacLab/isaaclab.sh -p` or an equivalent IsaacLab Python runtime
- rerun Task 4 with `scripts/inspect_openarm_isaac_env.py` and require discovered metadata, not fallback metadata
- only after Task 4 passes, run Task 5 unimanual Isaac replay and Task 6 bimanual Isaac replay
- freeze evidence after Task 5 and Task 6 pass, or fail them explicitly with the real reason

Task numbering used here:

- Task 2: existing replay schema validator (`scripts/validate_quest_spatial_replay.py`)
- Task 3: existing replay processor / IK gate (`scripts/test_quest_replay_processor_gate.py`)
- Task 4: Isaac env inspection (`scripts/inspect_openarm_isaac_env.py`)
- Task 5: unimanual Isaac replay, skipped in the current run because Task 4 failed with `reason: openarm_isaac_unavailable`
- Task 6: bimanual Isaac replay, skipped in the current run because Task 4 failed with `reason: openarm_isaac_unavailable`

## Pass and fail criteria

### Task 4, Isaac env inspection

Pass when the report says `status: pass`, `reason: pass`, and the env is registered with discovered Isaac metadata, including `joint_names_discovered: true` and `tcp_frame_discovered: true`.

Fail when Isaac Lab is missing, OpenArm Isaac registration is missing, the env is unregistered, or discovered joint or TCP metadata cannot be read from Isaac.

Current Task 4 result: fail. Isaac Lab imports through `/home/syhai/IsaacLab/isaaclab.sh -p`, but OpenArm Isaac is missing, so the report says `status: fail`, `reason: openarm_isaac_unavailable`, `availability.openarm_isaac: missing`, `joint_names_discovered: false`, and `tcp_frame_discovered: false`.

### Task 5, unimanual Isaac replay

Current Task 5 result: skipped and not run. Task 4 failed with `reason: openarm_isaac_unavailable`, so unimanual Isaac replay is still blocked.

Pass when the Isaac replay gate runs on the GPU PC and the frozen thresholds are met:

- `max_tracking_error_m <= 0.03`
- `control_rate_hz >= 25`
- `collision_count == 0`
- `nan_count == 0`
- `disabled_drift_m <= 0.005`
- `stop_latency_s <= 0.5`

Fail when any threshold is missed, the Isaac env is missing, or the replay cannot be run honestly.

### Task 6, bimanual Isaac replay

Current Task 6 result: skipped and not run. Task 4 failed with `reason: openarm_isaac_unavailable`, so bimanual Isaac replay status is not explicit.

Pass only when bimanual replay is explicit, both arms are covered, and the same frozen thresholds are met for the bimanual trace.

Fail when only one arm is covered, the bimanual trace is incomplete, the env is missing, or any threshold is missed.

## Transition rule to real hardware

Do not run safe shutdown, real motion, or recording gates until Task 5 unimanual passes and Task 6 bimanual status is explicit.

Current rule application: real hardware remains blocked. No real hardware, CAN, recording, safe shutdown, `send_action`, policy-server promotion, or Hub upload is authorized from the current evidence.

## Notes for the next operator

- The frozen Quest contract lives in `docs/quest_isaac_real_control_contract.md`.
- `scripts/inspect_openarm_isaac_env.py` is the current Task 4 evidence source.
- The current blocker is missing OpenArm Isaac package availability inside the active IsaacLab Python runtime, not missing `isaaclab` itself.
- `scripts/validate_quest_recording_dry_run.py` is the current safe failure path for recording dry runs.
- `scripts/test_quest_replay_processor_gate.py` is the current replay and IK gate reference.
