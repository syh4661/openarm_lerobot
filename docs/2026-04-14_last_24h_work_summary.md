# 2026-04-14 Last 24h Work Summary

## Purpose

This document is a compact summary of the work done in this repository during the last 24 hours, with links to the more detailed write-up where needed.

For the deeper recording / shutdown investigation narrative, see:

- `docs/2026-04-14_openarm_recording_work_report.md`

---

## Executive summary

The last 24 hours were mostly about stabilizing the OpenArm + LeRobot recording workflow and tightening the recording operator surface.

Main outcomes:

1. Recording wrapper behavior was clarified and improved in `scripts/run_record.sh`
2. README and operator-facing docs were expanded so dataset upload and FPS overrides are easier to use
3. RGB/full recording presets were normalized to 30 FPS defaults
4. Camera left/right wrist labeling was corrected by fixing the RealSense serial-role mapping
5. Validation / audit helper scripts were restored into `scripts/`
6. Local safe follower/leader wrapper work was brought into this repo to improve disconnect safety without depending on invasive upstream edits

---

## What landed today

### 1. Recording wrapper and operator docs improved

Relevant commits:

- `7e9347a` — `fix: 템플릿 오버라이드 전달을 안전화`
- `d155176` — `docs: README에 데이터 취득 안내 정리`
- `22eb3da` — `docs: README에 녹화 사용 가이드 정리`
- `34d6669`, `724d2e3` — `feat : edit record.sh`

What changed:

- `scripts/run_record.sh` became a clearer thin wrapper around LeRobot recording
- nested camera override args such as `--robot.*.cameras.*.fps=...` are handled at the wrapper/config-generation layer instead of being forwarded blindly
- wrapper usage examples now cover Hub upload, dataset metadata, and per-camera FPS tuning
- README now explains the separation between:
  - wrapper positional args
  - environment overrides for Hub upload
  - raw LeRobot CLI overrides

Why it matters:

- the documented commands now line up better with the runtime behavior
- operators can tune recording jobs without editing preset JSON files by hand

### 2. Default recording presets adjusted to 30 FPS

Relevant commit:

- `40155e4` — `feat: 녹화 프리셋 기본 FPS를 30으로 조정`

Files affected:

- `configs/record_full.json`
- `configs/record_rgb.json`
- `configs/record_nocam.json`
- `configs/record_rgb_head_fixture.json`
- `configs/realsense_3cam_mapping.yaml`

Why it matters:

- the repo now documents and defaults to a faster standard path first
- operators still have a clear downgrade path to 20 / 15 FPS when camera stability is worse than expected

### 3. Camera wrist labeling fix applied

Relevant files touched across today's work:

- `configs/record_rgb.json`
- `configs/record_full.json`
- `configs/record_rgb_head_fixture.json`
- `configs/realsense_3cam_mapping.yaml`
- `docs/lerobot_openarm_record_command.md`

What changed:

- the stale RealSense serial-role mapping was corrected so `left_wrist` and `right_wrist` align with the actual physical cameras

Why it matters:

- future recorded datasets should have the correct wrist-camera semantics
- older datasets recorded before this mapping fix may still need manual caution during interpretation

### 4. Validation / audit entrypoints restored

Relevant commits:

- `126b7c7` — `fix: restore recording compatibility checker and head fixture`
- `c9992ef` — `fix: restore unified validator entrypoints`
- `dee80c1` — `fix: restore audit and rollout gate scripts`
- `a9aa907` — `test: 로컬 녹화 호환성 검증 조건 보강`

Files added or restored today include:

- `scripts/check_lerobot_recording_compatibility.py`
- `scripts/validate_unified_camera_semantic_registry.py`
- `scripts/validate_unified_dataset_contract.py`
- `scripts/validate_unified_derived_view_spec.py`
- `scripts/audit_reference_dataset.py`
- `scripts/validate_unified_data_collection_rollout_gate.py`
- `configs/record_rgb_head_fixture.json`

Why it matters:

- the repo regained key validation entrypoints for recording compatibility, dataset contract checks, and rollout gating

---

## Final shutdown-safety status from field validation

This line of work is no longer just in-progress.

What was implemented locally:

- `src/openarm_lerobot/safe_followers.py` now contains local safe OpenArm follower and leader wrappers
- `src/openarm_lerobot/__init__.py` exports those safe classes for runtime registration
- `scripts/run_record.sh` pre-imports `openarm_lerobot` before invoking `lerobot_record`
- recording presets now use:
  - `robot.type = safe_bi_openarm_follower`
  - `teleop.type = safe_bi_openarm_leader`

Field validation outcome:

- short real hardware recording runs confirmed that both follower and leader now power down correctly at the end of recording
- the earlier regression where safe leader types triggered a non-interactive calibration prompt was fixed by reusing the existing `openarm_leader` calibration namespace and syncing that loaded calibration into the Damiao bus object
- practical recording validation also showed that `--dataset.fps=15` looked noticeably more stuttery than the default 30 FPS path when camera FPS was left at the preset 30

Why the 15 FPS run looked worse:

- in LeRobot teleop recording, `dataset.fps` effectively sets the main record/control loop cadence, not just saved video metadata
- raising `dataset.fps` from 15 to 30 therefore increases how often the system reads teleop action, reads robot observation, sends robot action, and writes dataset frames
- on top of that, the tested path still kept the preset camera streams at 30 FPS and read them through `cam.read_latest()`
- that means the 15 Hz run combined a slower teleop/robot/data loop with a `30 FPS camera -> 15 Hz sampler` mismatch, which is more prone to skipped intermediate frames and uneven frame age
- the aligned 30/30 path therefore looks smoother in practice

Operational implication:

- if lower-rate recording is needed, tune camera FPS and dataset FPS together (`30/30 -> 20/20 -> 15/15`) instead of only lowering `dataset.fps`

Why this matters:

- the repo now keeps the shutdown-safety policy locally, without carrying a generic upstream `../lerobot` patch
- the validated path covers both sides of the OpenArm teleop stack, not just the follower

---

## Repo-level narrative for the day

If we compress the day into one sentence:

> The repo moved from “basic recording wrapper and scattered operator knowledge” toward “documented recording flow, corrected camera mapping, restored validation tools, and a field-validated local shutdown-safety path for both follower and leader.”

---

## Suggested next documentation follow-ups

If we continue documenting this line of work, the next high-value docs would be:

1. a short `safe_follower` runbook explaining when to use `safe_bi_openarm_follower`
2. a camera-mapping verification checklist before recording sessions
3. a one-page operator checklist for local-only recording vs Hub-upload recording
