# Quest Phase D.4–D.7 Yaw-Only Reference Translation — DEFERRED

**Date deferred**: 2026-05-08
**Reason**: Quest VR teleop refinement consumes too many cycles. Operator pivots to
improving the existing leader-arm system efficiency in a separate session. This plan is
recorded verbatim so it can be picked up later without re-deriving context.

## Decision context (recap)

- D.2 sweep showed Y/Z plane coupling under the current `coord_transform_vec=[2,1,-3,4]`.
- Codex initially concluded the bug was "missing reference orientation absorption" but
  the existing code already does `relative = inv(T_ref) @ T_now`, so the translation is
  already in calibration-time controller frame (ref-local), not world frame.
- That ref-local frame depends on operator's controller pose at LG press. Pitch/roll
  errors at calibration leak directly into Y/Z plane coupling — exactly the D.2 symptom.
- Operator cannot reliably hold the controller perfectly horizontal at every reset.
- The only absolute axis is gravity (Z). Robot's central column is parallel to gravity,
  with both arms mounted horizontally on it.
- **Selected fix policy**: yaw-only reference absorption. Capture only the yaw component
  of controller pose at reset. Pitch/roll discarded so gravity Z is preserved. Operator's
  facing direction (yaw) is preserved so "operator forward = robot forward" remains
  natural across body re-orientation.

## Final approved plan (use as-is when work resumes)

### Summary

- D.4 is reframed as a frame-judgment audit: not "is `inv(ref) @ raw` present" but
  "what frame does the resulting translation delta live in".
- Current hypothesis: `inv(T_ref) @ T_now` causes translation to come out in
  calibration-time controller frame (ref-local).
- D.5 replaces full ref-local translation with yaw-only absorption. Pitch/roll are NOT
  applied to translation; gravity Z is preserved.
- `coord_transform_vec` is NOT extended to 3x3 / plane rotation. It stays
  permutation+sign only.

### Key Changes

#### D.4 [AUTONOMOUS] — frame audit

- Audit `quest_teleop.py::compute_calibrated_delta()` math; write
  `/tmp/d4_reference_absorption_audit.md`.
- Table required: raw/ref pose, `relative = inv(ref) @ raw`, translation extraction,
  `coord_transform_vec` application, output frame at each step.
- `/tmp` probe to produce evidence:
  - `R_ref = Rx(30°)`, `raw_pos = ref_pos + [0, dy, 0]`
  - User-target frame expected: `[0, dy, 0]`
  - Current implementation predicted: `[0, dy*cos30, dy*sin30]`
- If D.4b is not confirmed as ref-local, halt D.5 and replan.

#### D.5 [AUTONOMOUS, conditional on D.4b = ref-local]

- Add yaw-only translation mode to `compute_calibrated_delta()`.
- Add `translation_reference_mode` config field.
  Default `ref_local` for backward compat. Use `yaw_only` in D.6 / F / H.
- Yaw-only math:

  ```
  delta_pos_world = raw_pos_world - ref_pos_world
  yaw_at_reset    = atan2(R_ref[1, 0], R_ref[0, 0])     # NO leading minus
  R_calib         = Rz(yaw_at_reset)                    # pitch/roll dropped
  translation_delta = R_calib.T @ delta_pos_world
  ```

  Then apply existing `coord_transform_vec` sign/permutation.

- Derivation sanity (record in plan / commit message):
  - `R_ref` column 0 = controller local +X axis expressed in world.
  - `atan2(R_ref[1,0], R_ref[0,0])` = controller yaw +θ around world +Z.
  - `R_calib.T` projects a world translation into the operator-local yaw frame
    (gravity Z preserved).

- Orientation delta: keep existing `inv(ref) @ raw` ref-local handling.
  When `zero_orientation_delta=true`, orientation is ignored as today.

- Commit: `feat(quest): use yaw-only reference for spatial translation`.

#### 1bf607f bookkeeping

- `fix(quest): keep zero-delta hold through delta miss` belongs to Phase B / zero-delta
  stabilization. Already merged. Not in the D.4–D.7 implementation scope.

### Tests

Add to `scripts/test_quest_processor_steps.py`:

- `R_ref = Rx(30°)`, `raw_pos = ref_pos + [0, dy, 0]` → `[0, dy, 0]` (Z leak = 0).
- `R_ref = Rz(45°)`, `raw_pos = ref_pos + [0, dy, 0]` → `[+dy*sin45, +dy*cos45, 0]`.
- `R_ref = Rz(90°)`, `raw_pos = ref_pos + [dx, 0, 0]` → `[0, -dx, 0]`.
- Zero-rotation regression: result unchanged.
- `compute_orientation=False` returns same translation result.

Verification:

```bash
python3 -m py_compile \
  src/openarm_lerobot/quest_teleop.py \
  src/openarm_lerobot/quest_spatial_teleop.py \
  scripts/test_quest_processor_steps.py
python3 scripts/test_quest_processor_steps.py --urdf assets/openarm_right.urdf
```

### Operator validation

#### D.6 [NEEDS_OPERATOR]

- Use `/tmp` temp config: copy source config, keep `coord_transform_vec=[2,1,-3,4]`,
  add only `translation_reference_mode=yaw_only`.
- After explicit `READY D.6 X` / `READY D.6 Y` / `READY D.6 Z`, run no-send 10 s axis
  sweep each.
- Operator holds controller naturally; deliberately uses a different calibration pose
  on each sweep at least once.
- Acceptance: `delta_unavailable < 1%`, `violating_cmd_frames = 0`,
  `dominant axis ratio > 0.7`.
- Output: `/tmp/d6_post_absorption_sweep.md`.

#### D.7 [CONDITIONAL]

- Trigger only if D.6 leaves `coupling > 0.3`.
- Generate exactly one sign/permutation candidate from D.6 data. No 3x3 / plane
  rotation.
- Temp file `/tmp/d7_axis_candidate.json`. Source config unchanged.
- Re-validate after `READY D.7` with the same no-send sweep.

### E / F / H

- **E**: After DROID integration, confirm latency stays within ±5% of baseline.
- **F**: After `READY F`, dual `candump`, init pose ramp, 5 s stationary, 5 s +X, 5 s
  diagonal, safe disconnect. Acceptance: `can1=0`, `violating_cmd_frames=0`,
  zero-delta jump = 0, latency p50/p95 within
  `/tmp/smooth_d_retry_1778238863.log` baseline + 30% / +50%.
- **H**: Only after F passes. Lock source config. Do not touch `joint_limits` —
  Phase A confirmed left config is already correct.

### Hard rules

- No 3x3 / plane rotation extension to `coord_transform_vec`. Operator cannot reproduce
  controller pose precisely; per-session calibration must be runtime-only.
- `runtime` translation/orientation split is allowed (yaw-only translation +
  ref-local orientation).
- Source `configs/record_quest_left_nocam.json` remains unchanged until Phase H.
- Each `[NEEDS_OPERATOR]` step waits indefinitely for explicit `READY <phase>`. No
  response = stop.
- Implementation only after operator types `GO IMPLEMENT`.

## Bookkeeping artifacts already in repo

| Commit  | Purpose |
|---------|---------|
| `1f00151` | `fix(quest): keep translation delta when orientation is ignored` |
| `47e3d46` | `fix(quest): log calibration transform wait reasons` |
| `c5742de` | `feat(quest): add closed-loop timing diagnostics` |
| `1b16ff8` | `test(quest): DROID and zero-delta hold regressions` |
| `fc807b1` | `feat(quest): optional joint slew limiter` |
| `5f7111a` | `fix(quest): zero-delta hold uses last command` |
| `34a89a8` | `feat(quest): DROID-style EE velocity controller` |
| `1bf607f` | `fix(quest): keep zero-delta hold through delta miss` |

Phases A / B / C are complete and merged. D.4 audit, D.5 yaw-only impl, D.6 sweep,
optional D.7, E/F/H all remain.

## Resume checklist (when picking this back up)

1. Read this doc end-to-end.
2. Confirm `configs/record_quest_left_nocam.json` `joint_2` limit is still
   `[-90.0, 9.0]` (Phase A finding).
3. Spawn Codex in PLAN MODE; paste the plan body above.
4. Wait for Codex to ask `READY` for each operator phase. Do not let Codex skip them.
5. Verify the yaw_at_reset sign in any plan revision: it must be
   `+atan2(R_ref[1,0], R_ref[0,0])`, NOT negated. The negative form was an early
   Codex draft error.
6. After Phase H, return to mainline (bimanual setup + camera + pi0.5).
