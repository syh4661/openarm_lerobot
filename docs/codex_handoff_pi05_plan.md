# Codex Handoff — pi0.5 Box-Can Demo Pipeline

**Target agent**: Codex (or any code-executing LLM)
**Working dir**: `/home/syhlabtop/workspace/openarm_lerobot`
**Host**: `syhlabtop@10.252.216.81` (real OpenArm robot, both arms wired)
**Deadline**: 2026-05-14
**Anchor docs**: `docs/syhlabtop_handover.md` (7-day plan), `AGENTS.md` (env + style)

---

## Read-me-first

You are picking up where a previous session left off. Mission: collect Quest-teleoperated bimanual demos of "pick soda can, place in box", fine-tune **pi0.5**, deploy on the real robot. Read `docs/syhlabtop_handover.md` and `AGENTS.md` before touching anything.

This document is the **executable plan**. It encodes facts that were verified at session start (2026-05-07) and that your sandbox cannot rediscover quickly. Treat verified facts as authoritative; treat conjectures (marked ❓) as hypotheses to test.

**Hard rules**:
- Do **not** bypass `_handshake()` or torque-disable safeties.
- Do **not** delete or rewrite existing calibration JSON under `~/.cache/huggingface/lerobot/calibration/`.
- Do **not** push to `data/` paths that already contain demo episodes (`data/openarm_*` already-populated dirs are real artifacts).
- One-axis-at-a-time when tuning `coord_transform_vec` — never multi-axis edits.
- Live tests on the real arm: spatial_scale ≤ 0.3, max_ee_step_m ≤ 0.03 until proven smooth, e-stop in operator's hand.
- No live run is allowed until the preceding no-send gate passes and the human operator replies with an explicit `GO Phase N`.
- Do not pipe `yes` or otherwise auto-answer calibration prompts. A prompt means stop, inspect the current arm pose, and get an explicit operator decision.

---

## Current status (2026-05-07)

- Phase 0 code fixes are already present in the working tree: forced logging, `ROS_PACKAGE_PATH` bootstrap, `QUEST_DEBUG` joint-command logs, and non-empty no-send dry-run output.
- `rightTrig` is confirmed as the gripper command path: `_map_trigger_to_gripper_deg()` maps trigger `0..1` into the configured gripper range, then `QuestSpatialTeleop` normalizes it into `quest.gripper`. During axis tests, avoid trigger motion unless testing gripper deliberately.
- Test script signatures are verified:
  - `scripts/test_quest_ik_roundtrip.py --urdf <path> --target-frame <frame>`
  - `scripts/test_quest_processor_steps.py --urdf <path>`
- Phase 1/2 no-send translation-axis validation passed before the bus-map correction. Reinterpret that evidence as a **right Quest controller -> physical left arm (`can0`)** result, not the final desired controller/arm pairing.
- A 5s stationary no-send test at `spatial_scale=0.01`, `max_ee_step_m=0.005` passed with `violating_cmd_frames=0`. This makes a fixed IK seed/calibration failure less likely; saturation appears motion-axis/sign dependent.
- Physical `+X` failed on the source mapping `[-2, -1, -3, 4]` with `joint_1` saturation, then passed with the SO(3)-valid sign candidate `[-2, +1, +3, 4]`.
- Physical `+Y` saturated `joint_1` across sign and simple axis-swap candidates. Code inspection confirmed Quest `rot_delta` is not ignored in the closed-loop processor path: `MapQuestActionToRobotAction` emits `target_w*`, `EEReferenceAndDelta` applies it to the target pose, and IK solves the full pose. A new `QuestSpatialTeleopConfig.zero_orientation_delta` option is available for translation-only no-send isolation.
- Best-known candidate from that accidental right-controller-to-left-arm path is `coord_transform_vec: [2.0, 1.0, -3.0, 4.0]`, `zero_orientation_delta: true`, `spatial_scale: 0.1`, `max_ee_step_m: 0.02`. Treat it as a candidate for final left/right configs until no-send axis validation re-runs with the intended controller.
- Root cause of the earlier `joint_1=-80 deg` saturation was the upstream LeRobot `RobotKinematics` solving only a frame task for a 7-DOF arm. `src/openarm_lerobot/kinematics.py` now adds `OpenArmKinematics`, a local subclass with a weak placo posture task (`posture_weight=0.01`) so the 1-DOF redundancy stays near home without patching LeRobot.
- Posture IK validation:
  - 5s stationary no-send: `violating_cmd_frames=0`, clipped=0, `joint_1` range `0.000121 deg` (`/tmp/posture_stationary_1778165423.log`).
  - 20s axis no-send: `violating_cmd_frames=0`, clipped=0, `joint_1` range `0.000413 deg` (`/tmp/posture_axis_1778165467.log`).
  - B1 scale no-send (`spatial_scale=0.05`, `max_ee_step_m=0.01`): `violating_cmd_frames=0`, clipped 36/754 ~= 4.8% (`/tmp/posture_b1_1778165538.log`).
  - B2 scale no-send (`spatial_scale=0.1`, `max_ee_step_m=0.02`): `violating_cmd_frames=0`, clipped 5/851 ~= 0.6% (`/tmp/posture_b2_1778165613.log`).
- `SafeOpenArmFollower.send_action()` now rejects joint-limit violations before LeRobot's clipping path, logs the payload at ERROR, torque-disables the arm, and raises. Continue from Phase 3 preparation. Do not run live until the operator reviews the final no-send evidence and replies with explicit `GO Phase 3 live`.

---

## Verified facts (do not re-litigate)

1. **Env activation** (must run all three):
   ```bash
   source /home/syhlabtop/workspace/openarm_lerobot/.venv312/bin/activate
   source /home/syhlabtop/workspace/openarm_lerobot/scripts/env_rsusb_py312.sh
   export PYTHONPATH="/home/syhlabtop/workspace/openarm_lerobot/src:$PYTHONPATH"
   ```
   `pyproject.toml` has empty `dependencies = []`; the venv is **not** an editable install. The shipped `.venv` (no 312 suffix) is wrong — ignore it.

2. **ROS_PACKAGE_PATH bootstrap**: URDFs use `package://openarm_description/...`. `scripts/record_quest_closed_loop.py` now auto-adds `/home/syhlabtop/workspace` to `ROS_PACKAGE_PATH` when the sibling `openarm_description` package exists. Keep the explicit export below as a harmless operator fallback if debugging outside the script:
   ```bash
   export ROS_PACKAGE_PATH="/home/syhlabtop/workspace:${ROS_PACKAGE_PATH:-}"
   ```

3. **CAN bus map**: `can0` = physical **left** arm, `can1` = physical **right** arm. Verified by direct LED ENABLE/DISABLE tests on 2026-05-08; see `/tmp/bus_arm_mapping.md` and `/tmp/bus_arm_decision.md`. Both arms use motor IDs `0x01..0x08` and recv IDs `0x11..0x18`; arm identity comes from CAN bus isolation, not motor ID differences. `can2`/`can3` are leader/unused in this workflow unless explicitly configured. Diagnostic recipe in case of dropped motor:
   ```python
   # /tmp/scan_can.py — see git history this session for the full script
   ```

4. **Hardware lesson learned**: a "fault" red LED on a single motor that survives power-cycle usually means the motor controller is permanently faulted (was `joint_2` on the original right arm). Resolution this session: physical arm swap. Both arms now respond on can0/can1, but the verified map is `can0=left`, `can1=right`.

5. **Calibration**: `SafeOpenArmFollower` reuses calibration from the non-`safe_*` namespace via `_reuse_calibration_namespace` (`src/openarm_lerobot/safe_followers.py:163`). Existing files:
   - `~/.cache/huggingface/lerobot/calibration/robots/openarm_follower/openarm_right_follower.json`
   - `~/.cache/huggingface/lerobot/calibration/robots/openarm_follower/openarm_bimanual_follower_{left,right}.json`
   - `~/.cache/huggingface/lerobot/calibration/teleoperators/openarm_leader/...`
   On `connect()`, if calib file matches motor NVRAM → uses it + calls `set_zero_position()` (writes current pose as zero). If mismatch → interactive prompt: ENTER to keep file, `c` to recalibrate.

6. **Quest reader**: `oculus_reader.OculusReader` (in `/home/syhlabtop/workspace/droid/droid/oculus_reader/`) uses logcat (not socket forwarding). APK `com.rail.oculus.teleop` is installed and auto-launched. Quest must be **awake** (head sensor active or display on) AND controllers must be **held / not asleep** for transforms to flow. Calibration timeout is 10s (`QUEST_OPENARM_CALIBRATE_TIMEOUT_S`).

7. **Closed-loop script status**:
   - `scripts/record_quest_closed_loop.py` now uses `logging.basicConfig(..., force=True)`.
   - It bootstraps `ROS_PACKAGE_PATH` when the sibling `openarm_description` package exists.
   - `QUEST_DEBUG=1` emits spatial tracking and final commanded joint angles.
   - `--dry-run --control-time-s 30 --no-send-action` has produced non-empty episodes in `/tmp`.
   - Dataset path collisions remain real: re-running with the same generated TS or repo_id throws `FileExistsError` (`dataset_metadata.py:621`). Always generate fresh `repo_id` + `root` per run.

8. **Config files**:
   - `configs/record_quest_left_nocam.json` — left Quest controller driving the physical left arm on `can0`, no cameras. The robot `id` remains `openarm_right_follower` temporarily for calibration-file continuity; clean this namespace deliberately later. Must pass no-send axis validation before live.
   - `configs/record_quest_right_nocam.json` — right Quest controller driving the physical right arm on `can1`, no cameras. Uses `id: openarm_right_follower_can1_unvalidated` so it cannot silently reuse the left-arm legacy calibration. Must pass calibration review and no-send axis validation before live.
   - `configs/record_full.json` — bimanual with cameras. Use as the structural template for the bimanual Quest config you must create.
   - `configs/realsense_3cam_mapping.yaml` — RealSense serials.
   - **Camera serials** (per `docs/syhlabtop_handover.md`): right wrist `230322273311`, left wrist `315122270766`, chest `234322070493`.

---

## Phase 0 — Tooling fixes

**Status**: implemented. Keep this section as the acceptance checklist if regressions appear.

**Goal**: make `record_quest_closed_loop.py` debuggable and self-bootstrapping. Small surgical edits only.

### 0.1 Add `force=True` to logging.basicConfig
File: `scripts/record_quest_closed_loop.py` line ~217.
Change:
```python
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    force=True,
)
```
Apply the same fix to `scripts/debug_quest_input_only.py` (same pattern around line ~70).

### 0.2 Add ROS_PACKAGE_PATH bootstrap
File: `scripts/record_quest_closed_loop.py`, near top imports. Mirror the pattern from `scripts/validate_quest_recording_dry_run.py` exactly (same condition, same env mutation). Do not invent a new helper.

### 0.3 Add joint-command observability
Inside `record_loop` callsites in the script (or via a guarded log inside `make_processors` output), emit per-tick (gated on env `QUEST_DEBUG=1`):
- Quest controller pose (4x4 matrix, just translation row is fine)
- Calibrated EE delta
- Final commanded joint angles (8 values, deg)
- Whether action was clipped by `max_ee_step_m`
Use `_log_quest_debug` (already defined in `src/openarm_lerobot/quest_teleop.py:453`). If a clean spot to inject doesn't exist, add one wrapper in the script — do **not** edit `record_loop` in lerobot.

### 0.4 Diagnose silent dry-run
Re-run after 0.1–0.3:
```bash
QUEST_DEBUG=1 python -u scripts/record_quest_closed_loop.py \
  --config <fresh-tmp-config> --no-send-action --dry-run --control-time-s 10 \
  2>&1 | tee /tmp/dryrun_phase0.log
```
Confirm:
- "Starting Quest closed-loop runtime" line appears
- per-tick QUEST_DEBUG lines appear (≥ 100 over 10s at 30fps)
- exit 0
- `<root>/data/chunk-000/episode_000000.parquet` (or wherever the dataset writes) **exists** with non-zero rows

If parquet is still empty: drill into `record_loop` to find why frames don't reach `dataset.add_frame`. Likely `events["exit_early"]` flips — instrument the events dict.

### 0.5 Log analysis helper
Use the repo parser instead of ad-hoc parsing:
```bash
python3 scripts/analyze_quest_closed_loop_log.py /tmp/phase1.log \
  --joint-limits-config configs/record_quest_right_nocam.json
```

### 0.6 Commit
One commit per fix, conventional prefix:
- `fix(scripts): force=True on basicConfig so closed-loop logs surface`
- `fix(scripts): bootstrap ROS_PACKAGE_PATH for record_quest_closed_loop`
- `feat(quest): per-tick QUEST_DEBUG observability in closed-loop runner`
- `tools(quest): add closed-loop QUEST_DEBUG log analyzer`

**Done-when**: dry-run produces visible logger output AND a non-empty parquet AND exits cleanly.

---

## Phase 1 — IK dry-run validation (Step 1 of handover)

**Goal**: confirm joint commands are sane before any motor moves under teleop.

Operator: at the keyboard with Quest right controller in hand. Robot powered on, torque enabled, but `--no-send-action` will block writes.

Calibration gate before connect:
- Confirm the intended zero pose with the human operator. The current physical pose seen this session is vertically hanging with the gripper open; older prompt text may say "closed". Do **not** assume which one is correct for this swapped arm.
- If the script shows a calibration prompt, stop and ask the operator whether to keep existing calibration or recalibrate. Do not auto-send ENTER or `c`.
- If no prompt appears, still record in the log/notes which physical zero pose was used.

```bash
TS=$(date +%s)
python3 -c "
import json
cfg = json.load(open('configs/record_quest_right_nocam.json'))
cfg['dataset']['repo_id'] = f'local/dryrun_${TS}'
cfg['dataset']['root'] = f'/tmp/openarm_dryrun_${TS}'
cfg['dataset']['num_episodes'] = 1
cfg['dataset']['episode_time_s'] = 30
cfg['teleop']['spatial_scale'] = 0.03
cfg['teleop']['max_ee_step_m'] = 0.01
json.dump(cfg, open('/tmp/qcfg.json','w'), indent=2)
"
QUEST_DEBUG=1 timeout 75 python -u scripts/record_quest_closed_loop.py \
  --config /tmp/qcfg.json --no-send-action --dry-run --control-time-s 30 \
  2>&1 | tee /tmp/phase1.log
```

Test protocol — operator narrates each step out loud while moving Quest:
1. Hold right grip (`RG`) for the full 30s. Releasing RG resets the controller reference and invalidates the axis window.
2. **0–10s** translate Quest +X (forward, away from operator)
3. **10–20s** translate Quest +Y (left in operator's reference)
4. **20–30s** translate Quest +Z (up)
5. Do not press `rightTrig` during translation-axis mapping; it drives gripper.

Then analyze `/tmp/phase1.log`:
```bash
python3 scripts/analyze_quest_closed_loop_log.py /tmp/phase1.log \
  --joint-limits-config configs/record_quest_right_nocam.json
```

Gate criteria:
- Each 10s window has enough `spatial_tracking` rows to identify the commanded direction.
- `clipped` is 0 or rare enough to explain; if clipped frequently, lower movement amplitude or scale.
- `violating_cmd_frames` is 0, ignoring only sub-microdegree floating-point noise already covered by the parser tolerance.
- Dataset episode parquet exists and has non-zero rows.

Record findings in `/tmp/phase1_axis_map.md`. Do not yet edit `coord_transform_vec`.

Empirical axis-mapping loop:
- Use `spatial_scale=0.01`, `max_ee_step_m=0.005`.
- Run one 5s no-send test per physical Quest motion axis. The operator holds `RG` and moves only that axis.
- For translation-axis isolation, set `teleop.zero_orientation_delta=true` in the temp config. Without that, natural controller rotation is propagated into `target_w*` and can dominate IK behavior.
- If a test saturates, stop and try flipping the sign for that same `coord_transform_vec` entry first.
- Only try swapping axis numbers after sign flip still fails.
- Keep the send-action/live safety gate as backlog for Phase 3 entry; do not solve it inside Phase 1 unless live readiness is otherwise reached.

**Operator gate**: after the mapping table is written, ask for explicit `GO Phase 2`.

**Done-when**: a documented mapping table {Quest axis → Robot direction observed} for X, Y, Z, confirmation no joint command exceeds limits in `configs/record_quest_right_nocam.json` joint_limits, and explicit operator `GO Phase 2`.

---

## Phase 2 — `coord_transform_vec` tuning

**Goal**: produce a `coord_transform_vec` value that maps Quest motion intuitively (Quest forward → robot forward, Quest left → robot left, Quest up → robot up).

`_coord_vec_to_matrix` is at `src/openarm_lerobot/quest_teleop.py:401`. Each entry semantics:
- 4 values map to robot output `[x, y, z, gripper]`
- Magnitude `1/2/3/4` selects Quest axis source (1=x, 2=y, 3=z, 4=gripper)
- Sign flips that axis

Procedure:
1. Define convention (write it down once, do not change later):
   - Robot frame: `+x` forward, `+y` left, `+z` up (operator-facing).
   - Quest frame from Phase 1 measurements.
2. Decide each output index sequentially. Test each candidate in a fresh `/tmp/qcfg_phase2_*.json` first.
3. Re-run Phase 1's dry-run protocol. Verify only the changed axis behaves correctly.
4. After one axis is verified, ask the operator to approve locking that index. Only then edit ONE value in `configs/record_quest_right_nocam.json`.
5. Final sanity: full circle motion in horizontal plane → robot EE traces the same circle (handedness preserved).

Start from the safe values validated in Phase 1 (`spatial_scale=0.03`, `max_ee_step_m=0.01`) and increase only after no-send stability is clear. For live readiness, final source config must still satisfy `spatial_scale ≤ 0.3`, `max_ee_step_m ≤ 0.03`.

**Commit** after each axis is locked:
- `tune(quest): right coord_transform_vec[0] -> X axis verified`
- ... etc.

**Operator gate**: ask for explicit `GO Phase 3`.

**Done-when**: `coord_transform_vec` in `configs/record_quest_right_nocam.json` produces intuitive mapping for all 3 translation axes + gripper. `spatial_scale` and `max_ee_step_m` settled at safe values. Operator has replied `GO Phase 3`.

---

## Phase 3 — Single-arm final-controller live tests (Step 3 of handover)

**Goal**: first time the real arm moves under teleop, with operator hand on e-stop.

Pre-flight:
- Workspace clear of obstacles, no humans within arm reach.
- E-stop in operator's free hand, tested.
- `spatial_scale ≤ 0.3`, `max_ee_step_m ≤ 0.03` (verify in config!).
- Camera off. Use `record_quest_left_nocam.json` for left controller -> left arm, or `record_quest_right_nocam.json` for right controller -> right arm.
- Robot in safe rest pose.
- Confirm gripper behavior: `rightTrig` commands gripper. If unpressed trigger would put the gripper in an unsafe state for the test, resolve that before live.
- Operator must explicitly say `GO Phase 3 Run 2 LEFT` or `GO Phase 3 Run 2 RIGHT` after reviewing that arm's final no-send log and the bus mapping decision.

```bash
TS=$(date +%s)
# fresh config (same as Phase 1 but no --no-send-action)
QUEST_DEBUG=1 timeout 30 python -u scripts/record_quest_closed_loop.py \
  --config /tmp/qcfg_phase3_${TS}.json --control-time-s 10 \
  2>&1 | tee /tmp/phase3_run1.log
```

Operator: 10 seconds of small, slow Quest motions. Stop instantly on any unexpected jerk.

After run: review log for any `clipped` flags from `max_ee_step_m`, joint-limit hits, or torque warnings. Check robot is still at sane pose.

**Failure handling**:
- Sudden lurch: hit e-stop, capture log, lower `max_ee_step_m` to 0.02, retry only after operator review.
- Any motor goes red mid-run: power-cycle that arm only after operator confirms.
- If joint limit clipping happens: `coord_transform_vec` may still be off — go back to Phase 2.

Iterate `control-time-s` 10 → 20 → 30 once smooth. Then bump `spatial_scale` 0.3 → 0.5 → 0.7 if operator wants more responsiveness, but only if no clipping.

**Operator gate**: ask for explicit `GO Phase 4`.

**Done-when**: 30s continuous teleop session, no e-stop, no joint-limit clipping, no abnormal logs. Operator subjectively comfortable with mapping and has replied `GO Phase 4`.

---

## Phase 4 — Bimanual Quest implementation

**Goal**: add the code/assets/config needed for `configs/record_quest_bimanual_nocam.json` without mixing this work into the right-arm live gate.

Known blockers:
- `QuestSpatialTeleopConfig` currently rejects anything except `controller_side="right"`.
- `record_quest_closed_loop.py` currently constructs only `SafeOpenArmFollower`, not `SafeBiOpenArmFollower`.
- `assets/openarm_right.urdf` exists; a left-arm kinematics asset does not.
- The bimanual Quest config does not exist yet.

Implementation sequence, with regression tests after each step:

1. **Teleop/controller support**
   - File: `src/openarm_lerobot/quest_spatial_teleop.py`
   - Add support for a left controller or a bimanual composition only after inspecting the existing `quest_teleop.py` / `quest_processor.py` patterns.
   - Preserve the right-arm `quest_spatial_teleop` contract and run:
     ```bash
     python3 scripts/test_quest_processor_steps.py --urdf assets/openarm_right.urdf
     python3 scripts/test_quest_ik_roundtrip.py --urdf assets/openarm_right.urdf --target-frame openarm_hand_tcp
     ```

2. **Closed-loop bimanual robot branch**
   - File: `scripts/record_quest_closed_loop.py`
   - Add a `safe_bi_openarm_follower` branch that constructs `SafeBiOpenArmFollower` and creates per-arm kinematics/processors.
   - Keep the existing right-only branch unchanged and verify right-only no-send still works before connecting both arms.

3. **Left URDF asset**
   - Source: sibling `openarm_description` xacro. Start by searching:
     ```bash
     rg -n "left|right|mirror|side|prefix|xacro" /home/syhlabtop/workspace/openarm_description/urdf
     ```
   - Generate the left asset with the repo's xacro invocation, not by hand-editing URDF XML.
   - Expected destination: `assets/openarm_left.urdf`.
   - Verify:
     ```bash
     python3 scripts/test_quest_ik_roundtrip.py --urdf assets/openarm_left.urdf --target-frame openarm_hand_tcp
     ```

4. **Bimanual no-camera config**
   - File: `configs/record_quest_bimanual_nocam.json`
   - Start from `configs/record_full.json` for robot structure and camera-free `configs/record_nocam.json` for no-camera shape.
   - Use right-arm tuned values from Phase 2/3. For left, start with a mirror candidate but tune it in Phase 4.5; do not assume it is correct.

Structural template: `configs/record_full.json` (the bimanual + cameras config). Copy:
- `robot.type` = `safe_bi_openarm_follower`
- `robot.left_arm_config.port` = `can0`, `right_arm_config.port` = `can1`
- Per-arm `motor_config`, `position_kp`, `position_kd`, `joint_limits` (left arm matches the currently tuned single-arm config; right arm needs separate validation)

Teleop side: bimanual variant of `quest_spatial_teleop`. Check `src/openarm_lerobot/quest_spatial_teleop.py` for whether `controller_side="bimanual"` is supported, or if you need separate left + right teleops both fed from one Quest reader. If the latter, look for `bi_quest_*` or similar in `quest_teleop.py` / `quest_processor.py`. Add the smallest compatible abstraction; do not change right-only behavior.

Initial values:
- `left_arm.coord_transform_vec` = candidate `[2.0, 1.0, -3.0, 4.0]`, but retune with the left Quest controller.
- `right_arm.coord_transform_vec` = candidate `[2.0, 1.0, -3.0, 4.0]` or mirror candidate. Tune separately with the right Quest controller before live; do not assume it is correct.
- `spatial_scale` = whatever Phase 3 settled on.

### 4.5 Per-arm tuning
Repeat the dry-run + axis-tuning + small-live-test cycle for both final pairings: left Quest controller -> left arm on `can0`, and right Quest controller -> right arm on `can1`. Expect mirror candidates may work; verify, do not assume.

**Operator gate**: ask for explicit `GO Phase 5`.

**Done-when**: `configs/record_quest_bimanual_nocam.json` runs `--no-send-action` cleanly with both arms tracking, then a 10s live bimanual test with `spatial_scale 0.3` succeeds, and the operator replies `GO Phase 5`.

**Commit**: `feat(configs): bimanual Quest nocam config + tuned coord transforms`

---

## Phase 5 — Camera-enabled bimanual config

**Goal**: `configs/record_quest_bimanual.json` (full = bimanual + 3 cameras), since pi0.5 needs vision.

Take Phase 4's nocam config, add the camera blocks from `configs/record_full.json`:
- right wrist: RealSense serial `230322273311`
- left wrist: RealSense serial `315122270766`
- chest: RealSense serial `234322070493`

Camera FPS guidance from `AGENTS.md`:
- Start at 30/30/30 (matches dataset.fps=30)
- If RSUSB bandwidth degrades (frame drops > 1%), step down: 20/20/20 then 15/15/15.
- See `docs/realsense_rsusb_runbook.md`.

Preflight check before any teleop run:
```bash
python3 scripts/check_lerobot_recording_compatibility.py --preset full --run-name preflight
```
Confirm 3-camera streams open and frames flow.

**Operator gate**: ask for explicit `GO Phase 6`.

**Done-when**: 20s teleop session with 3 cameras streaming, parquet has 600 frames (20s × 30fps), 3 mp4 sidecar files exist, no frame drops > 1%, and the operator replies `GO Phase 6`.

**Commit**: `feat(configs): bimanual Quest config with 3 RealSense cameras`

---

## Phase 6 — Demo data collection

**Goal**: ≥ 30 successful episodes of "pick soda can, place in box".

Setup:
- Soda can and box visible to chest camera.
- Operator practices the task once (`--no-send-action` first if helpful, then live).
- Use `scripts/run_record.sh` workflow with the bimanual full preset, **but** the wrapper currently targets the leader-follower flow not Quest. **Decision point**: extend `run_record.sh` to take a `quest_full` preset, OR just call `record_quest_closed_loop.py` directly with the bimanual config and HF push env.

Direct call recipe:
```bash
TS=$(date +%s)
OPENARM_RECORD_REPO_ID=KETI-IRRC/openarm_bimanual_box_can OPENARM_RECORD_PUSH_TO_HUB=1 \
  python -u scripts/record_quest_closed_loop.py \
  --config configs/record_quest_bimanual.json \
  --control-time-s 20
```

Discipline:
- Mark each episode pass/fail in operator's notebook (or a sidecar `notes.csv`).
- Only push **successful** episodes to the dataset. Failed = re-record.
- Reset between episodes: arm to home, can/box to start positions, ~20s.
- Collect in batches of 10. Spot-check a parquet after each batch:
  ```bash
  python3 -c "
  import pandas as pd
  df = pd.read_parquet('data/<repo>/data/chunk-000/episode_000005.parquet')
  print(df.columns.tolist(), len(df))
  "
  ```

**Operator gate**: ask for explicit `GO Phase 7` before starting training from the collected dataset.

**Done-when**: 30+ episodes pushed to `KETI-IRRC/openarm_bimanual_box_can` (or chosen repo_id), each ~600 frames, 3 cameras, all 16 joint actions populated, and the operator replies `GO Phase 7`.

---

## Phase 7 — pi0.5 fine-tune

**Goal**: a deployable policy checkpoint.

Run on **syhai GPU PC**, not syhlabtop. SSH from syhlabtop or move there. The dataset will be pulled from HuggingFace if pushed; else mount/copy.

```bash
# on syhai
lerobot train \
  --policy.type=pi0 \
  --dataset.repo_id=KETI-IRRC/openarm_bimanual_box_can \
  --output_dir=outputs/pi0_box_can_$(date +%Y%m%d) \
  --batch_size=8 \
  --num_workers=4 \
  --steps=30000
```

Monitor:
- Loss convergence (track every 1000 steps).
- VRAM utilization.
- Save best-by-loss checkpoint plus final.

**Operator gate**: ask for explicit `GO Phase 8` before any autonomous real-robot deployment.

**Done-when**: a checkpoint that on a held-out episode predicts actions with reasonable MSE (define threshold once first checkpoint exists; no point pre-committing a number), and the operator replies `GO Phase 8`.

---

## Phase 8 — Deployment + verification

**Goal**: robot performs the task autonomously.

Architecture: policy server on syhai GPU, action client on syhlabtop. See `scripts/run_policy_server.sh` and `docs/gpu_server_setup.md`.

Steps:
1. Start policy server on syhai pointing at the trained checkpoint.
2. On syhlabtop, run a deploy script that connects to `safe_bi_openarm_follower`, opens cameras, queries the policy server every tick, sends actions through the **same safety wrappers** used in teleop (`SafeBiOpenArmFollower` torque-disable on disconnect, joint limits, max-step clamps).
3. Place can + box in the start configuration (same as demo collection).
4. Operator hand on e-stop. Run the policy.
5. Capture video + per-tick action log for analysis.

If task fails: collect failure case, possibly augment dataset and retrain.

**Done-when**: 3 consecutive successful runs of the box-can task autonomously.

---

## Cross-cutting concerns

### Safety (every phase)
- Operator e-stop within reach for all live runs.
- First live run after any config change uses `--control-time-s 5`, low scale.
- Watch for joint LED states between runs. Any red → stop, diagnose.
- The previous arm got physically damaged from the last tuning attempt. Be paranoid.

### Calibration drift
- If a run starts with the arm not at the expected zero pose, `set_zero_position()` will write the wrong zero. Always confirm pose before connect. If unsure: type `c` at the calibration prompt to redo.

### Dataset path collisions
- Always generate `repo_id` and `root` with `$(date +%s)` suffix per run, until production collection in Phase 6.
- Do NOT delete existing dirs under `data/`. The temp configs can write to `/tmp/openarm_dryrun_*` freely.

### Existing data preserved (do not touch)
- `data/openarm_record_TEMPLATE/meta/info.json` — leftover from 2026-04-17, harmless but may break dry-run dataset creation. Worked around by always overriding `dataset.root` in temp configs.
- All `~/.cache/huggingface/lerobot/calibration/*.json` — these are the calibrations from prior physical setups. Treat as read-only. New calibrations write fresh files alongside.

### Diagnostic recipes (keep handy)
- CAN motor scan: see "Verified facts" §3.
- Quest reader sanity (without robot):
  ```bash
  python -c "
  from openarm_lerobot.quest_spatial_teleop import QuestSpatialTeleop, QuestSpatialTeleopConfig
  import json
  raw = json.load(open('configs/record_quest_right_nocam.json'))
  t = dict(raw['teleop']); [t.pop(k, None) for k in ('type','initial_joint_seed_deg','motor_names','joint_offsets_deg','urdf_path')]
  tl = QuestSpatialTeleop(QuestSpatialTeleopConfig(**t))
  tl.connect(calibrate=False)
  import time; time.sleep(2)
  print(tl._reader.get_transforms_and_buttons())
  tl.disconnect()
  "
  ```
- Verify env imports:
  ```bash
  python -c "import openarm_lerobot, numpy, can, lerobot, placo; print('ok')"
  ```

### When to ask the human
- Before swapping arms or any irreversible hardware change.
- Before pushing to a public/shared HuggingFace repo.
- If joint-limit clipping persists across more than 2 retunes — design issue, not a code issue.
- If silent record_loop bug in Phase 0.4 turns out to be deep in lerobot — propose fix, get approval before patching upstream.

---

## What NOT to do

- Don't add caching/abstraction layers "for later" — surgical edits only.
- Don't write new test files unless the user asks; ad-hoc python `-c` checks are fine.
- Don't `git add -A` — stage explicit files. The repo has untracked private dirs.
- Don't push to `master` without operator confirmation per commit batch.
- Don't bypass the calibration prompt with brute-force `c` if you don't understand current arm pose.

---

## Quick session resume snippet

If a fresh agent picks this up later, paste this to start:
```
You are continuing the pi0.5 box-can demo project on /home/syhlabtop/workspace/openarm_lerobot.
Read docs/codex_handoff_pi05_plan.md and docs/syhlabtop_handover.md.
Run the env activation block from "Verified facts" §1.
Run the env imports check at the bottom of "Diagnostic recipes".
Then resume from the phase whose Done-When clause is not yet satisfied.
```
