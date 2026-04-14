# 2026-04-14 OpenArm Recording Work Report

## Scope of today's work

This report summarizes the full debugging and implementation session around OpenArm + LeRobot recording on 2026-04-14.

Main themes:

1. Gripper / end-effector not behaving correctly during teleop recording
2. Dataset push-to-Hugging-Face confusion and wrapper usability issues
3. Camera left/right labeling mismatch in recorded datasets
4. Follower shutdown safety bug where the end effector could remain armed after recording stopped
5. Migration of the shutdown fix from upstream `../lerobot` into this repo as a local OpenArm-specific wrapper
6. Extending the local shutdown path to the leader side and validating the full hardware de-arm behavior in the field

---

## Initial problems observed

### 1. Gripper not moving during recording

Observed during:

- `./scripts/run_record.sh rgb openarm_dualmanip_test03 ...`

Relevant symptoms:

- arm control path ran successfully
- recording completed successfully
- gripper/end-effector appeared non-responsive

Initial hypothesis was that leader and follower gripper conventions might be mismatched, especially around the gripper value sign and clipping range.

### 2. Dataset push looked broken

At first it seemed that the dataset was not uploading to Hugging Face.

After inspection, this turned out not to be a code bug. The earlier command was simply running with:

- `dataset.push_to_hub = false`
- `repo_id = local/...`

So nothing was supposed to upload in that run.

### 3. Camera left/right labels were reversed in the uploaded dataset

The uploaded dataset showed left and right wrist camera content swapped.

This turned out not to be a `bi_openarm_follower` merge-order bug. The code preserves configured camera names correctly. The real issue was stale D405 serial-to-role mapping in the preset files.

### 4. Follower did not safely power down after recording

Most critical issue found today.

Observed symptoms:

- recording exited
- disconnect logs were printed
- follower gripper / end effector could remain active
- green motor LED stayed on
- user confirmed leader was off, but follower end effector was still resisting backdrive

This was treated as a real safety issue, not just a cosmetic logging issue.

### 5. Later field validation result

After the local safe wrapper path was extended and re-tested, the final hardware validation outcome was:

- follower powers down correctly
- leader powers down correctly
- both sides now de-arm at recording shutdown in the validated local path

---

## Investigation summary

## A. Recording wrapper and config generation

Inspected:

- `scripts/run_record.sh`
- generated files under `.tmp/record_*.json`
- `configs/record_rgb.json`
- `configs/record_full.json`

Findings:

- `run_record.sh` is a thin wrapper that stamps a preset JSON into `.tmp/...json`
- actual behavior is controlled downstream by `lerobot_record.py`
- the wrapper originally documented camera leaf overrides that LeRobot itself did not accept as CLI arguments

### Problem found in wrapper CLI behavior

This command failed:

```bash
--robot.left_arm_config.cameras.left_wrist.fps=20
--robot.left_arm_config.cameras.chest.fps=20
--robot.right_arm_config.cameras.right_wrist.fps=20
```

Reason:

- LeRobot accepts replacing `cameras` as a whole dict
- it does **not** accept nested leaf flags like `...cameras.left_wrist.fps=20`

### Fix applied

`scripts/run_record.sh` was updated so that:

- camera leaf override args are intercepted by the wrapper
- those values are merged directly into the generated JSON config
- those camera args are not forwarded to `lerobot_record.py`

Result:

- the documented wrapper examples now match actual behavior
- per-camera FPS overrides work at the wrapper layer

---

## B. Hugging Face push behavior

### What was confirmed

The upload path in `../lerobot/src/lerobot/scripts/lerobot_record.py` is correct:

- dataset finalization happens
- robot disconnect happens
- then `dataset.push_to_hub(...)` happens if `cfg.dataset.push_to_hub` is true

### What was learned

To actually upload, the wrapper must run with:

```bash
OPENARM_RECORD_REPO_ID=<namespace>/<dataset_name>
OPENARM_RECORD_PUSH_TO_HUB=1
```

Authentication was also checked and Hugging Face login was valid.

### Concept clarification completed

Clarified the relationship between:

- `run_name`
- `dataset.root`
- `dataset.repo_id`
- Hugging Face dataset naming

Important distinction:

- `run_name` controls local folder naming
- `repo_id` controls logical dataset identity / Hugging Face destination

### Wrapper usability improvement

`scripts/run_record.sh` help output was updated to include a full real-world example containing:

- Hub push enabled
- explicit repo id
- task string
- 20 FPS dataset setting
- per-camera FPS overrides

---

## C. Camera mapping investigation and fix

### Files inspected

- `configs/record_rgb.json`
- `configs/record_full.json`
- `configs/record_rgb_head_fixture.json`
- `configs/realsense_3cam_mapping.yaml`
- `docs/lerobot_openarm_record_command.md`
- `../lerobot/src/lerobot/robots/bi_openarm_follower/bi_openarm_follower.py`

### Key finding

`bi_openarm_follower` already preserves camera names as configured.

So the left/right swap seen in datasets came from stale physical camera serial mapping, not from observation merge logic.

### Mapping before fix

- `left_wrist` -> `230322273311`
- `right_wrist` -> `315122270766`

### Mapping after fix

- `left_wrist` -> `315122270766`
- `right_wrist` -> `230322273311`
- `chest` remains `234322070493`

### Files updated

- `configs/record_rgb.json`
- `configs/record_full.json`
- `configs/record_rgb_head_fixture.json`
- `configs/realsense_3cam_mapping.yaml`
- `docs/lerobot_openarm_record_command.md`

### Outcome

Future recordings should now label wrist cameras correctly.

Note: datasets recorded before this fix may still contain incorrect left/right wrist labeling.

---

## D. Gripper / shutdown safety investigation

### What was ruled out

1. **Push-to-Hub is not the reason the follower stays armed**
   - `robot.disconnect()` occurs before `dataset.push_to_hub(...)`

2. **Runtime source mismatch was not the issue**
   - recording was confirmed to import from the sibling repo source tree:
     - `/home/syhlabtop/workspace/lerobot/src/lerobot/...`

3. **The shutdown function was not being skipped entirely**
   - disconnect logs did appear

### Core safety issue identified

The real issue was that a clean-looking software shutdown did not reliably guarantee hardware de-arming.

The investigation found multiple failure modes along the way:

- generic disconnect path too thin
- failure could be hidden by weak response expectations
- OpenArm hardware patterns suggested that the vendor path expects a batched disable/receive sequence, not a simplistic per-motor one-shot assumption

---

## E. Intermediate approaches attempted today

## 1. Upstream `lerobot`-side patches (later reverted)

We temporarily patched sibling repo files such as:

- `../lerobot/src/lerobot/motors/damiao/damiao.py`
- `../lerobot/src/lerobot/robots/bi_openarm_follower/bi_openarm_follower.py`

These changes included:

- retrying torque disable
- raising errors on safe shutdown failure
- forcing both arms to attempt disconnect even if one failed first

Why this was not kept as the final solution:

- too invasive to upstream LeRobot / generic Damiao layer
- user preferred not to rely on direct upstream motor-source modification if avoidable
- OpenArm-specific shutdown policy belongs more naturally in the OpenArm integration layer

These upstream edits were reverted so the final solution lives in `openarm_lerobot` instead.

## 2. First local safe wrapper attempt

Created local OpenArm-specific wrappers in this repo:

- `src/openarm_lerobot/safe_followers.py`

Added:

- `SafeOpenArmFollower`
- `SafeBiOpenArmFollower`
- corresponding config subclasses

Initial local strategy:

- send strict disable command motor-by-motor
- require a response per motor
- then call `bus.disconnect(False)`

This improved the safety semantics but was still not enough in practice for the end effector.

---

## F. Final local wrapper architecture implemented today

### Goal

Keep the fix local to `openarm_lerobot` and avoid widening impact into generic LeRobot / Damiao code.

### Mechanism chosen

Use LeRobot plugin/discovery support to register local robot types from this repo.

### Files added / changed

#### New local wrapper module

- `src/openarm_lerobot/safe_followers.py`

Contains:

- `SafeOpenArmFollowerConfig`
- `SafeBiOpenArmFollowerConfig`
- `SafeOpenArmLeaderConfig`
- `SafeBiOpenArmLeaderConfig`
- `SafeOpenArmFollower`
- `SafeBiOpenArmFollower`
- `SafeOpenArmLeader`
- `SafeBiOpenArmLeader`

#### Package root export

- `src/openarm_lerobot/__init__.py`

This file now exports the safe follower and leader classes directly so LeRobot’s generic factory can instantiate them from the package root.

#### Wrapper runtime loading

- `scripts/run_record.sh`

Updated to:

- add `$ROOT/src` to `PYTHONPATH`
- import `openarm_lerobot` before calling `lerobot_record.main()`

This ensures LeRobot imports and registers the local custom follower/leader types before config parsing / instantiation, without relying on an unsupported extra CLI flag.

#### Preset robot type migration

Updated presets to use the new local safe types:

- `configs/record_rgb.json`
- `configs/record_full.json`
- `configs/record_nocam.json`
- `configs/record_rgb_head_fixture.json`

Preset role changes:

- `robot.type = safe_bi_openarm_follower`
- `teleop.type = safe_bi_openarm_leader`

### Calibration regression found and fixed

Switching to safe leader types initially caused a new issue:

- recording could prompt for leader calibration in a non-interactive run and exit with `EOFError`

Root cause:

- safe leader objects had their calibration file path redirected to the legacy `openarm_leader` namespace
- but the internal `DamiaoMotorsBus` still held the original empty calibration reference created before that redirect

Fix applied:

- reuse the legacy OpenArm calibration namespace for both safe follower and safe leader classes
- explicitly sync the loaded calibration back into the bus object after redirecting the calibration path

Result:

- safe types preserve the original OpenArm calibration files
- non-interactive recording no longer needs to recalibrate just because the safe type name changed

### Field validation result

Validated with real recording runs after the local safe wrapper migration:

- follower shutdown: PASS
- leader shutdown: PASS
- end-of-recording de-arm behavior: PASS

Changed from:

- `bi_openarm_follower`

to:

- `safe_bi_openarm_follower`

---

## G. Plugin/discovery issue encountered and fixed

### Problem

After first introducing local safe wrappers, runtime failed with:

```text
Could not locate device class 'SafeBiOpenArmFollower' for config 'SafeBiOpenArmFollowerConfig'
```

### Root cause

The config class had registered correctly, but LeRobot’s generic factory searches for the device class on the imported package root.

We had imported the module, but had not exposed:

- `SafeBiOpenArmFollower`
- `SafeOpenArmFollower`

from `openarm_lerobot.__init__`.

### Fix

`src/openarm_lerobot/__init__.py` was updated to export those classes directly.

### Validation

Confirmed:

- package import succeeds
- registry contains:
  - `safe_openarm_follower`
  - `safe_bi_openarm_follower`
- generated recording config now shows:
  - `robot.type: safe_bi_openarm_follower`

---

## H. Stronger shutdown fix implemented at the end of the session

### Important finding from vendor OpenArm examples

Vendor `openarm_can` examples consistently shut down using a pattern like:

1. `disable_all()`
2. short wait
3. `recv_all()`

Examples inspected:

- `openarm_can/examples/demo.cpp`
- `openarm_can/setup/motor_check.cpp`
- `openarm_can/python/examples/test_gripper_posforce.py`

### Why this mattered

Our first local safe wrapper still used a per-motor disable/ack path. That was likely too weak or too timing-sensitive for the actual OpenArm hardware behavior, especially at the gripper/end effector.

### New shutdown behavior now implemented

`src/openarm_lerobot/safe_followers.py` was updated so `_safe_disable_all_motors()` now:

1. builds disable messages for all motors
2. sends disable to all motors first
3. waits briefly
4. collects responses for all expected recv IDs in batch using `_recv_all_responses(...)`
5. processes all received responses
6. fails if any motor did not acknowledge disable
7. retries the whole batch if needed

This is much closer to the vendor OpenArm disable pattern than the old one-by-one path.

### Why this is likely better

- aligns with actual OpenArm examples
- reduces chance that the gripper/end-effector is missed by a too-short per-motor response window
- gives a single explicit failure path if any motor fails to acknowledge shutdown

---

## Files changed today in `openarm_lerobot`

### New files

- `src/openarm_lerobot/safe_followers.py`
- `docs/2026-04-14_openarm_recording_work_report.md` (this report)

### Modified files

- `scripts/run_record.sh`
- `configs/record_rgb.json`
- `configs/record_full.json`
- `configs/record_rgb_head_fixture.json`
- `configs/realsense_3cam_mapping.yaml`
- `docs/lerobot_openarm_record_command.md`
- `src/openarm_lerobot/__init__.py`

### Upstream `../lerobot` changes that were intentionally reverted

Any earlier shutdown-related edits in sibling `../lerobot` were reverted. Final intended solution is local to this repo.

---

## Validation completed today

### Wrapper / config validation

- generated config files correctly reflect camera serial swap
- generated config files correctly reflect safe robot type
- wrapper now accepts and applies per-camera leaf overrides
- help output includes full practical recording example

### Plugin / runtime validation

- `openarm_lerobot` package import works with `PYTHONPATH=$ROOT/src`
- LeRobot sees safe custom robot types once discovery path is provided
- safe follower classes are now exported at package root for factory instantiation
- `safe_followers.py` compiles successfully

### Hugging Face behavior validation

- confirmed push-to-hub path is functional when enabled
- confirmed earlier “push failure” was configuration misunderstanding rather than broken code

### Camera semantic validation

- confirmed preset-level camera mapping is now:
  - left wrist = `315122270766`
  - right wrist = `230322273311`
  - chest = `234322070493`

### Recording smoothness / FPS validation

One more practical observation was confirmed during real runs:

- recording with `--dataset.fps=15` looked noticeably stuttery
- 30 FPS recording looked materially smoother

### Why 15 Hz looked worse in this repo

This did **not** look like a simple “RealSense 15 FPS is always broken” issue.

The more accurate explanation from the code path is that `dataset.fps` in LeRobot is not just a passive video metadata knob. In teleop recording it effectively sets the **main record/control loop cadence**.

So when the user raised `dataset.fps` from 15 to 30 and things felt much better, that was not merely “saving video at a higher FPS.” It also raised how often the loop:

- reads robot observations
- reads teleoperator actions
- sends actions to the robot
- writes dataset frames

The camera-vs-dataset mismatch still matters, but it is a secondary factor on top of that higher-level loop-rate change.

Relevant paths:

- `scripts/run_record.sh`
- `configs/record_rgb.json`
- `configs/record_full.json`
- `configs/record_rgb_head_fixture.json`
- `configs/realsense_3cam_mapping.yaml`
- `../lerobot/src/lerobot/scripts/lerobot_record.py`
- `../lerobot/src/lerobot/robots/openarm_follower/openarm_follower.py`
- `../lerobot/src/lerobot/cameras/camera.py`
- `../lerobot/src/lerobot/cameras/opencv/camera_opencv.py`
- `../lerobot/src/lerobot/cameras/realsense/camera_realsense.py`
- `../lerobot/src/lerobot/datasets/lerobot_dataset.py`
- `../lerobot/src/lerobot/datasets/video_utils.py`

### Code-path explanation

1. The OpenArm presets default camera streams to **30 FPS**.
2. The wrapper forwards `--dataset.fps=15` as a dataset / record-loop override unless camera leaf overrides are also given.
3. `lerobot_record.py` passes `cfg.dataset.fps` directly into `record_loop(... fps=cfg.dataset.fps, ...)`.
4. Inside `record_loop`, `control_interval = 1 / fps` and each loop tick performs:
   - `robot.get_observation()`
   - `teleop.get_action()`
   - `robot.send_action()`
   - `dataset.add_frame()`
5. OpenArm robot observations read camera frames through `cam.read_latest()`.
6. `read_latest()` is a non-blocking “give me the latest buffered frame” path, not a synchronous “wait for a fresh frame exactly now” path.
7. Dataset timestamps / encoded video cadence are then written against the nominal dataset FPS.

That means the tested 15 FPS run was effectively closer to:

- teleop / robot / dataset loop cadence: **15 Hz**
- camera production: **30 FPS**
- recording / dataset frame cadence: **15 FPS**
- frame retrieval style: **latest buffered frame**

So two effects were stacked together:

1. the robot control / capture loop itself was slower
2. the camera stream was still being sampled from a faster 30 FPS producer

### Practical consequence

At 15 Hz, the recorder only advances the full teleop/robot/data loop every ~66.7 ms while the cameras are still producing frames every ~33.3 ms.

In that setup, two things can happen more easily:

1. intermediate camera frames are skipped
2. the recorder may observe frames with uneven age because it is peeking the latest available buffered image rather than synchronizing to a fresh capture edge

That already makes teleop and observation updates feel less continuous. When encoded back onto a clean 15 FPS timeline, the result can look more “jerky” than expected from FPS reduction alone.

By contrast, 30 FPS looks smoother here because:

- preset camera FPS is already 30
- the teleop / robot / dataset loop target is also 30
- the camera stream cadence and recording cadence are better aligned
- there is less visible aliasing between camera production and dataset sampling

### Important nuance

This does **not** prove that “15 FPS is bad in general.”

It means that in the observed runs, **15 Hz teleop/robot/data cadence with 30 FPS camera streams** produced a visibly worse result than the aligned 30/30 path.

If a lower-rate mode is required, the more correct comparison is:

- `dataset.fps = 15`
- camera FPS also overridden to `15`

rather than only lowering dataset FPS.

### Operational takeaway

For this repo, the safest tuning order remains:

1. try `30 / 30`
2. if needed, degrade to `20 / 20`
3. only then test `15 / 15`

In other words, lower the dataset cadence and camera cadence **together**, not just the dataset side.

---

## Current unresolved / follow-up items

## 1. Ctrl-C traceback behavior vs hardware state

At one point a `KeyboardInterrupt` traceback appeared after cleanup/upload. Later observation suggested the traceback itself was not the main problem; the real concern was whether the end effector had actually powered down.

So the shutdown safety problem took priority over cosmetic interrupt handling.

If hardware shutdown becomes stable, graceful `KeyboardInterrupt` suppression can be revisited afterward.

## 2. Old datasets may still contain wrong wrist labels

Any datasets recorded before the camera serial remap fix may have left/right wrist images mislabeled.

Those should be treated carefully.

---

## Recommended immediate next tests

1. Run a very short record session using the current local safe follower path
2. Stop with Ctrl-C
3. Verify on hardware:
   - left arm fully relaxes
   - right arm fully relaxes
   - gripper / end effector can be backdriven
   - green LED no longer indicates active hold
4. If shutdown still fails, capture:
   - exact terminal output
   - whether left/right/only gripper remains armed
   - whether failure is symmetric on both arms

If failure persists after the latest batch-disable change, the next step should likely be:

- implement an even more OpenArm-specific vendor-style safe-off bridge
- or call a dedicated OpenArm-side disable-all sequence rather than reproducing it through raw Damiao messaging

---

## Final status at end of session

### Fixed / improved

- wrapper camera leaf override handling
- wrapper help text usability
- Hugging Face push understanding and usage
- camera left/right serial mapping
- local OpenArm-specific safe follower plugin architecture
- local safe robot discovery and instantiation path
- stronger batch-based shutdown sequence aligned with vendor examples

### Still awaiting hardware confirmation

- complete and reliable end-effector de-arming on real shutdown under all exit paths
