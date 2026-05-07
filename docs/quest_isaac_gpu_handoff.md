# Quest Isaac GPU Handoff

## Current status

This repo is the tracked source of truth for the next agent. `.sisyphus/` plans, evidence, and notepads are ignored after clone, so this document is the handoff that survives a fresh checkout.

Current run update, 2026-05-07:

- Isaac Lab was found at `/home/syhai/IsaacLab/isaaclab.sh`. Running with `/home/syhai/IsaacLab/isaaclab.sh -p` imports `isaaclab`, while `omni.isaac.lab` fails with `ModuleNotFoundError`.
- Task 4 was rerun with the IsaacLab interpreter after the AppLauncher-first inspector fix. The report is an honest fail with `status: fail` and `reason: env_unavailable`: OpenArm Isaac now imports and registers as `availability.openarm_isaac: available:openarm`, the env is available, and `Isaac-Reach-OpenArm-v0` appears in the matching env list, but Isaac-derived metadata discovery did not pass.
- Task 4 failed the metadata gate because Isaac-derived joint names and the TCP frame were not discovered. The warning summary is safe metadata inspection only: env config parsing failed on a missing `source` module, so fallback contract joint and TCP values remain context only and must not be promoted as discovered metadata.
- Task 5 and Task 6 were skipped and not run because Task 4 did not pass metadata discovery. Task 12 and Task 13 were also skipped and not run for the same gate failure.
- Task 2 processor evidence contains accepted fail reports for both right-axis and right-failure traces because the active command environment is missing the `lerobot` runtime dependency.
- Task 3 Quest input-only evidence reached Quest reader setup and failed to resolve `OculusReader`; no robot action path was used.
- Real hardware remains blocked because Task 4 metadata discovery failed, Task 5 unimanual has not passed, and Task 6 bimanual status is not explicit.
- No real hardware, CAN, recording, safe shutdown, `send_action`, or Hub upload is authorized by this handoff.

Current evidence paths:

- `.sisyphus/evidence/task-9-isaac-imports.log`: IsaacLab wrapper exists, `isaaclab` import succeeds, `omni.isaac.lab` import fails.
- `.sisyphus/evidence/task-4-isaac-env.json`: Task 4 machine-readable status, reason, availability, matching env names, and metadata discovery booleans.
- `.sisyphus/evidence/task-4-isaac-env.log`: Task 4 stdout, stderr, and exit code capture. The wrapper can exit 0 even when the JSON report says fail, so the JSON report status and reason are authoritative.
- `.sisyphus/evidence/task-2-processor-right.json`: Task 2 right-axis processor accepted fail report, blocked by missing `lerobot`.
- `.sisyphus/evidence/task-2-processor-failure.json`: Task 2 right-failure processor accepted fail report, blocked by missing `lerobot`.
- `.sisyphus/evidence/task-3-quest-input-only.log`: Task 3 Quest input-only setup failed at `OculusReader` resolution before any robot action path.

Completed pushed commits:

- `4528747` `docs(quest): freeze isaac real control contract`
- `7d7bd5a` `test(quest): validate spatial replay fixtures`
- `aa9f014` `test(quest): gate replay through processor ik path`
- `77f830b` `test(isaac): inspect local openarm environment`
- `2cffd6b` `test(quest): validate dry run recording path`
- `a7a26da` `fix(init): defer safe_followers import to avoid eager can dependency`

Current local machine status:

- Isaac Lab itself is present through `/home/syhai/IsaacLab/isaaclab.sh -p`, and OpenArm Isaac imports and registers in that runtime after AppLauncher startup.
- `scripts/inspect_openarm_isaac_env.py` reports `status: fail` with `reason: env_unavailable` because Isaac-derived joint names and the TCP frame were not discovered.
- Real hardware is not ready on this machine, and it must stay blocked until Task 5 unimanual passes and Task 6 bimanual status is explicit.
- Historical warning notes still matter for later hardware work: Placo neutral URDF self collision warnings, SocketCAN was not properly shut down after a failed CAN connect, and no permission issue was observed.

## Required environment on the GPU PC

- clone this repo
- sibling `../lerobot/src`
- sibling `../openarm_description`
- Isaac Lab and the local OpenArm Isaac package in the same active environment
- Python 3.12 runtime, `.venv312` or equivalent
- `ROS_PACKAGE_PATH=/path/to/workspace`
- current blocker to fix before Task 5 or Task 6: make Isaac-derived OpenArm metadata discoverable from the registered env, especially joint names and the TCP frame

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

For the current installed IsaacLab path, Task 4 was run through `/home/syhai/IsaacLab/isaaclab.sh -p`. That interpreter imports Isaac Lab and, after AppLauncher startup, imports and registers OpenArm Isaac. Task 4 still cannot pass until the registered env yields discovered joint names and TCP frame metadata.

## Next implementation tasks on the GPU PC

- fix the active Isaac environment or inspector path so the registered OpenArm Isaac env exposes Isaac-derived joint names and TCP frame metadata
- rerun Task 4 with `scripts/inspect_openarm_isaac_env.py` and require discovered metadata, not fallback metadata
- only after Task 4 passes, run Task 5 unimanual Isaac replay and Task 6 bimanual Isaac replay
- freeze evidence after Task 5 and Task 6 pass, or fail them explicitly with the real reason

Task numbering used here:

- Task 2: existing replay schema validator (`scripts/validate_quest_spatial_replay.py`)
- Task 3: existing replay processor / IK gate (`scripts/test_quest_replay_processor_gate.py`)
- Task 4: Isaac env inspection (`scripts/inspect_openarm_isaac_env.py`)
- Task 5: unimanual Isaac replay, skipped in the current run because Task 4 failed with `reason: env_unavailable` from metadata discovery failure
- Task 6: bimanual Isaac replay, skipped in the current run because Task 4 failed with `reason: env_unavailable` from metadata discovery failure
- Task 12 and Task 13: skipped and not run because Task 4 did not pass metadata discovery

## Pass and fail criteria

### Task 4, Isaac env inspection

Pass when the report says `status: pass`, `reason: pass`, and the env is registered with discovered Isaac metadata, including `joint_names_discovered: true` and `tcp_frame_discovered: true`.

Fail when Isaac Lab is missing, OpenArm Isaac registration is missing, the env is unregistered, or discovered joint or TCP metadata cannot be read from Isaac.

Current Task 4 result: fail. Isaac Lab imports through `/home/syhai/IsaacLab/isaaclab.sh -p`, and OpenArm Isaac imports and registers as `availability.openarm_isaac: available:openarm`. The env is available and `Isaac-Reach-OpenArm-v0` is listed, but the report says `status: fail`, `reason: env_unavailable`, `joint_names_discovered: false`, and `tcp_frame_discovered: false` because Isaac-derived metadata was not discovered.

### Task 5, unimanual Isaac replay

Current Task 5 result: skipped and not run. Task 4 failed with `reason: env_unavailable` because metadata discovery did not pass, so unimanual Isaac replay is still blocked.

Pass when the Isaac replay gate runs on the GPU PC and the frozen thresholds are met:

- `max_tracking_error_m <= 0.03`
- `control_rate_hz >= 25`
- `collision_count == 0`
- `nan_count == 0`
- `disabled_drift_m <= 0.005`
- `stop_latency_s <= 0.5`

Fail when any threshold is missed, the Isaac env is missing, or the replay cannot be run honestly.

### Task 6, bimanual Isaac replay

Current Task 6 result: skipped and not run. Task 4 failed with `reason: env_unavailable` because metadata discovery did not pass, so bimanual Isaac replay status is not explicit.

Pass only when bimanual replay is explicit, both arms are covered, and the same frozen thresholds are met for the bimanual trace.

Fail when only one arm is covered, the bimanual trace is incomplete, the env is missing, or any threshold is missed.

## Transition rule to real hardware

Do not run safe shutdown, real motion, or recording gates until Task 5 unimanual passes and Task 6 bimanual status is explicit.

Current rule application: real hardware remains blocked. No real hardware, CAN, recording, safe shutdown, `send_action`, policy-server promotion, or Hub upload is authorized from the current evidence.

## Notes for the next operator

- The frozen Quest contract lives in `docs/quest_isaac_real_control_contract.md`.
- `scripts/inspect_openarm_isaac_env.py` is the current Task 4 evidence source.
- The current blocker is failed OpenArm Isaac metadata discovery inside the active IsaacLab runtime, not missing `isaaclab` or missing OpenArm Isaac registration.
- `scripts/validate_quest_recording_dry_run.py` is the current safe failure path for recording dry runs.
- `scripts/test_quest_replay_processor_gate.py` is the current replay and IK gate reference.
