# PROJECT KNOWLEDGE BASE

**Generated:** 2026-05-04  
**Commit:** 5a6a901  
**Branch:** master

## MANDATORY LLM CODING BEHAVIOR GUIDELINES

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

These rules apply to coding, refactoring, documentation, and verification work in this repository.

Tradeoff: These guidelines bias toward caution over speed. For trivial tasks, use judgment.

1. Think Before Coding

- Think through the task before making changes. Check the surrounding code, existing patterns, and likely edge cases.
- Prefer verification over assumption. Use tools to confirm what the repo actually does.
- Don't assume. Don't hide confusion. Surface tradeoffs.
- If the request is ambiguous, pause and clarify before editing.

Example:

- If a function looks wrong, inspect the caller, tests, and nearby conventions before changing it.
- If the task can be solved in two ways, note the tradeoff and choose the simplest correct path.

2. Simplicity First

- Choose the simplest correct solution.
- Avoid unnecessary abstractions, extra layers, and speculative cleanup.
- No features beyond what was asked.
- If you write 200 lines and it could be 50, rewrite it.

Example:

- Prefer a direct fix over introducing a helper, wrapper, or framework.
- Prefer a small patch over a refactor unless the refactor is required to solve the issue cleanly.

3. Surgical Changes

- Keep edits focused on the stated goal.
- Do not expand the scope into unrelated refactors, feature work, or style churn.
- Preserve existing behavior unless the task explicitly requires a change.
- The test: Every changed line should trace directly to the user's request.

Example:

- If you only need to fix one branch of logic, do not rename unrelated symbols or reformat the whole file.
- If a change touches several files, each file should be necessary for the requested outcome.

4. Goal-Driven Execution

- Stay focused on the single requested outcome.
- Finish the work completely, then verify it.
- Do not stop after a partial implementation.
- If fewer unnecessary changes are made and the result is cleaner, these guidelines are working.

Example:

- If a test fails, fix the root cause and re-run the relevant verification instead of patching around the failure.
- If the requested outcome is not yet fully met, keep going until it is.

These guidelines are working if: fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

## OVERVIEW

`openarm_lerobot` is a thin OpenArm ↔ LeRobot integration layer for data acquisition, Quest teleoperation, safe OpenArm wrappers, and LeRobot-compatible dataset output. It is not a full LeRobot fork; it assumes sibling runtime repos and local hardware setup.

## STRUCTURE

```text
openarm_lerobot/
├── src/openarm_lerobot/  # Quest/OpenArm adapter code, safety wrappers, bridge client
├── scripts/             # recording launchers, validators, Quest debug/test tools, setup scripts
├── configs/             # editable recording presets and RealSense camera mapping
├── data/                # generated LeRobot datasets; artifact storage, not source code
├── docs/                # canonical operator/runbook docs
├── assets/              # URDF and robot assets used by teleop/IK
└── pyproject.toml       # minimal setuptools src-layout metadata
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Understand project scope | `README.md`, `docs/phase1_architecture.md` | Phase-1 data flow and baseline order |
| Record data | `scripts/run_record.sh`, `docs/lerobot_openarm_record_command.md` | Wrapper generates `.tmp/record_*` configs |
| Quest/OpenArm teleop | `src/openarm_lerobot/quest_teleop.py`, `quest_spatial_teleop.py` | Calibration, IK, state machine, action formatting |
| Safe robot wrappers | `src/openarm_lerobot/safe_followers.py` | CAN torque-disable shutdown path |
| Quest processor mapping | `src/openarm_lerobot/quest_processor.py` | Quest spatial keys → LeRobot robot-action schema |
| Camera setup | `configs/realsense_3cam_mapping.yaml`, `docs/realsense_rsusb_runbook.md` | RSUSB runtime, serials, FPS degradation path |
| Dataset/schema validation | `scripts/validate_unified_*.py`, `scripts/audit_reference_dataset.py` | CLI validators, not pytest tests |
| GPU policy server | `scripts/run_policy_server.sh`, `docs/gpu_server_setup.md` | External OpenPI/uv workflow |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `QuestOpenArmTeleop` | class | `src/openarm_lerobot/quest_teleop.py` | Main Quest-to-OpenArm teleop pipeline |
| `QuestOpenArmTeleopConfig` | class | `src/openarm_lerobot/quest_teleop.py` | Frozen right-arm Quest hardware contract |
| `QuestSpatialTeleop` | class | `src/openarm_lerobot/quest_spatial_teleop.py` | Spatial-action teleop variant for processor flow |
| `MapQuestActionToRobotAction` | class | `src/openarm_lerobot/quest_processor.py` | Maps Quest spatial action fields to robot actions |
| `SafeOpenArmFollower` / `SafeOpenArmLeader` | classes | `src/openarm_lerobot/safe_followers.py` | Single-arm safe wrappers |
| `SafeBiOpenArmFollower` / `SafeBiOpenArmLeader` | classes | `src/openarm_lerobot/safe_followers.py` | Bimanual safe wrappers |
| `OpenArmBridgeClient` | class | `src/openarm_lerobot/bridge_client.py` | WebSocket/msgpack client for remote policy inference |
| `validate_manifest` | function | `scripts/validate_unified_camera_semantic_registry.py` | Camera/semantic registry acceptance gate |

## CONVENTIONS

- Python package uses `src/` layout and Python `>=3.12`; `pyproject.toml` has no lint/test tool config.
- LeRobot is a sibling dependency (`../lerobot`), not vendored. OpenArm runtime remains in sibling OpenArm repos.
- Runtime scripts are launched directly from `scripts/`; no `console_scripts` are defined.
- Script prefixes are meaningful: `validate_`/`check_`/`audit_` verify, `test_` asserts, `debug_` observes, `record_`/`run_` execute, `bootstrap_`/`install_`/`env_` prepare machines.
- Canonical camera names are `chest`, `left_wrist`, `right_wrist`; `wrist_a`/`wrist_b` are transitional physical-mount names.
- Quest/OpenArm constants encode hardware assumptions directly: right controller, USB Quest reader, `openarm_hand_tcp`, seven OpenArm joints, gripper range `[-65, 0]`.
- Docs are operator runbooks. Root `session-*.md` files are transient logs; do not mine them verbatim for durable guidance.

## ANTI-PATTERNS (THIS PROJECT)

- Do not treat Quest `.vel` / `.torque` action fields as real measurements; they are placeholders.
- Do not call Quest `get_action()` before calibration.
- Do not relax dataset/camera validator equality checks unless the downstream schema intentionally changes.
- Do not hand-edit generated dataset artifacts under `data/`; regenerate through recording/validation flow.
- Do not assume `dataset.fps` changes camera FPS; camera rates live under `--robot.*.cameras.*.fps`.
- Do not open all three RealSense streams at unstable bandwidth; degrade from `30/30` to `20/20` to `15/15`.
- Do not ignore safe shutdown failures; torque-disable errors are hardware-control failures.

## COMMANDS

```bash
source /home/syhlabtop/workspace/openarm_lerobot/.venv312/bin/activate
source /home/syhlabtop/workspace/openarm_lerobot/scripts/env_rsusb_py312.sh
./scripts/run_record.sh <nocam|rgb|full> <run_name> [episodes] [episode_s] [reset_s] [lerobot overrides...]
OPENARM_RECORD_REPO_ID=KETI-IRRC/openarm_phase1_test12 OPENARM_RECORD_PUSH_TO_HUB=1 ./scripts/run_record.sh rgb test12 2 20 20
python3 scripts/check_lerobot_recording_compatibility.py --preset rgb --run-name compat
python3 scripts/validate_unified_dataset_contract.py --manifest <manifest.json> --report <report.json>
python3 scripts/validate_unified_camera_semantic_registry.py --manifest <manifest.json> --report <report.json>
python3 scripts/validate_unified_derived_view_spec.py --spec <spec.json> --report <report.json>
python3 scripts/validate_unified_data_collection_rollout_gate.py --manifest <manifest.json> --report <report.json>
python3 -m py_compile src/openarm_lerobot/*.py scripts/*.py
bash -n scripts/run_record.sh
bash -n scripts/run_policy_server.sh
```

## NOTES

- No `.github/workflows`, Makefile, Dockerfile, devcontainer, pytest config, ruff config, or mypy config were found.
- LSP diagnostics are noisy without local `can` and `lerobot.*` imports on the analysis path; import failures often reflect missing sibling/runtime environment, not necessarily changed code.
- `scripts/run_record.sh` and setup scripts contain machine-specific absolute paths under `/home/syhlabtop/workspace` and `/home/syhlabtop/src/librealsense`.
- Use Python 3.12 for LeRobot runtime; Python 3.10 appears only in legacy/RSUSB validation paths.
- Prefer canonical docs: `README.md`, `docs/phase1_architecture.md`, `docs/next_steps.md`, `docs/realsense_rsusb_runbook.md`, `docs/lerobot_openarm_record_command.md`.
