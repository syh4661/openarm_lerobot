# `scripts` KNOWLEDGE BASE

## OVERVIEW

Operational command surface: recording wrappers, standalone validators, Quest debug/test tools, policy-server launcher, and machine setup scripts.

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Record LeRobot data | `run_record.sh` | Main wrapper; generates `.tmp/record_*` config |
| Quest closed-loop recording | `record_quest_closed_loop.py` | Supports `--dry-run`, `--no-send-action` |
| Config-only compatibility | `check_lerobot_recording_compatibility.py` | Uses `OPENARM_RECORD_COMPAT_ONLY=1` |
| Dataset gates | `validate_unified_*.py`, `audit_reference_dataset.py` | Strict CLI validators |
| Quest debug | `debug_quest_*.py` | Operator-facing observation tools |
| Quest tests | `test_quest_*.py` | Direct asserts; not pytest |
| Runtime setup | `bootstrap_*`, `env_rsusb_*`, `install_*`, `rebuild_*` | Machine-specific prep |
| GPU policy server | `run_policy_server.sh` | External OpenPI + `uv` path |

## CONVENTIONS

- Prefixes encode intent: `validate_`/`check_`/`audit_` verify, `test_` asserts, `debug_` observes, `record_`/`run_` executes.
- Validators are non-mutating unless an explicit `--report` path is supplied.
- Test files are executable scripts with `main()` and direct assertions; no pytest fixtures or `conftest.py` exist.
- Shell scripts use strict mode; keep `set -euo pipefail` when editing.
- Root installers require `sudo`; do not silently downgrade that guard.

## ANTI-PATTERNS

- Do not run hardware-affecting Quest scripts without considering `--dry-run`, `--no-send-action`, or `--once` first.
- Do not bypass `run_record.sh` preset validation for `nocam|rgb|full`.
- Do not relax rollout gates: they require authoritative status and prerequisite evidence.
- Do not treat debug scripts as automated CI; they are operator tools.
- Do not add pytest-only assumptions unless you also add actual pytest config/tree.

## COMMANDS

```bash
./scripts/run_record.sh <nocam|rgb|full> <run_name> [episode_count] [episode_time_s] [reset_time_s] [lerobot overrides...]
python3 scripts/check_lerobot_recording_compatibility.py --preset rgb --run-name compat
python3 scripts/record_quest_closed_loop.py --config configs/record_quest_right_nocam.json --dry-run --no-send-action
python3 scripts/debug_quest_input_only.py --config configs/record_quest_right_nocam.json --once
python3 scripts/test_quest_timed_capture.py
python3 scripts/test_quest_processor_steps.py
python3 scripts/validate_unified_dataset_contract.py --manifest <manifest.json> --report <report.json>
python3 scripts/validate_unified_camera_semantic_registry.py --manifest <manifest.json> --report <report.json>
python3 scripts/validate_unified_data_collection_rollout_gate.py --manifest <manifest.json> --report <report.json>
bash -n scripts/run_record.sh
bash -n scripts/run_policy_server.sh
```

## NOTES

- `run_record.sh` writes generated configs under `.tmp/`.
- `OPENARM_RECORD_REPO_ID` and `OPENARM_RECORD_PUSH_TO_HUB` override dataset target/upload behavior.
- `run_policy_server.sh` clones/uses external OpenPI state; keep it separate from local validation gates.
