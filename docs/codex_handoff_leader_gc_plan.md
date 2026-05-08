# Codex Handoff — Bimanual Leader Joint-4 Gravity Compensation

**Status**: Plan locked, ready for GO IMPLEMENT in next session.
**Scope**: Master arm (leader) usability only. Parallel to Quest VR teleop work — must not touch Quest files.
**Working dir**: `/home/syhlabtop/workspace/openarm_lerobot`
**Anchor docs**: `docs/syhlabtop_handover.md`, `docs/codex_handoff_pi05_plan.md`, `AGENTS.md`

---

## Read-me-first

Operator wants the existing record pipeline (`./scripts/run_record.sh rgb <name> <eps> <ep_t> <reset_t>`) to keep working unchanged, with optional joint_4 (elbow) gravity feedforward torque on the leader arms so the operator's hand does not carry the full elbow gravity load. Other joints stay freewheel — operator can handle them by hand.

Activation path (no record-script edit):
```bash
OPENARM_RECORD_TEMPLATE_OVERRIDE=configs/record_rgb_gc.json \
  ./scripts/run_record.sh rgb test_gc 1 20 20
```

---

## Hard Rules

- **Quest files untouched**: `src/openarm_lerobot/quest_*.py`, `scripts/record_quest_*.py`, `scripts/test_quest_*.py`, `scripts/debug_quest_*.py`, `scripts/analyze_quest_closed_loop_log.py`, `configs/record_quest_*.json`.
- **LeRobot upstream untouched**: `/home/syhlabtop/workspace/lerobot/`.
- **Production record configs untouched**: `record_rgb.json`, `record_nocam.json`, `record_full.json`, `run_record.sh`.
- **Only joint_4 gets torque commands**. All other motors remain `disable_torque`.
- **Default `tau_scale=0.0`** — selecting `safe_*_leader_gc` type alone must reproduce existing freewheel behavior exactly.
- **Live ramp gated by explicit operator `READY GC.<N>` reply**. No live progression without it.
- **Hard tau cap `tau_clip_nm=0.5` Nm**. Increasing this requires a separate explicit GO from operator.
- **Both arms disabled on any fault**: bimanual wrapper must call `disable_torque` on both arms even if one arm errors.

---

## Locked decisions (operator-confirmed)

- **Q1** (right-arm mount): `[+90.0, 0.0, 0.0]` — mirror of left `[-90.0, 0.0, 0.0]` (extracted from `openarm_description/urdf/robot/v10.urdf.xacro`). Verified by Phase 0.3 sanity table.
- **Q2** (`tau_scale=1.0`): defer to a follow-up PR. This PR caps at `tau_scale=0.7`.
- **Q3** (`tau_clip_nm` increase): a separate explicit GO is required to raise the cap above 0.5 Nm.

---

## Phase 0 — Reference + Sanity (AUTONOMOUS, code only)

### 0.1 Reference summary

Sources to read once and summarize:
- `/home/syhlabtop/workspace/openarm_teleop/control/gravity_compasation.cpp` (KDL `ChainDynParam` + `Vector(0,0,-9.81)` + `JntToGravity`; torque sent in MIT torque field as Nm)
- `/home/syhlabtop/workspace/openarm_teleop/src/controller/dynamics.cpp`
- `/home/syhlabtop/workspace/lerobot/src/lerobot/robots/unitree_g1/unitree_g1.py:148,478` (`gravity_compensation` flag + `arm_ik.solve_tau(q)` published with `publish_lowcmd(tau=...)`)
- `/home/syhlabtop/workspace/openarm_lerobot/assets/openarm_right.urdf` (11 inertial blocks)
- `openarm_description/urdf/robot/v10.urdf.xacro` (mount rpy)

Output: `/tmp/leader_gc_reference_summary.md` — formulation, mount values, Nm convention, left/right mirror.

### 0.2 Pinocchio sanity

Pinocchio loads `assets/openarm_right.urdf` as `nq=9` (gripper finger prismatics included). Use only the 7 arm joints `openarm_joint1..7`; finger joints set to `q=0`.

For 7 representative poses, log `pin.rnea(model, data, q, zeros(nv), zeros(nv))[idx_v(openarm_joint4)]`:
- `q=[0]*7` (hanging zero) — expected `|tau4| < 0.1`
- `[0,0,0,π/2,0,0,0]` (elbow 90°)
- `[0,0,0,-π/2,0,0,0]`
- `[0,π/2,0,π/2,0,0,0]` (hands-up)
- shoulder forward + elbow 90° (forearm horizontal forward)
- shoulder side + elbow 90°
- 1 random pose

Sign is **not hardcoded** — Phase 0.3 mount sign and Phase 3 live response together pin it down.

Output: `/tmp/leader_gc_sanity_${TS}.md` — pose × `tau_4` table (with and without mount rotation).

### 0.3 Mount orientation

- Left mount: `[-90.0, 0.0, 0.0]` from `openarm_description/urdf/robot/v10.urdf.xacro`.
- Right mount: `[+90.0, 0.0, 0.0]` (mirror, locked Q1).
- Apply via `model.gravity.linear = R(mount_rpy).T @ [0, 0, -9.81]`.

If 0.2 sanity contradicts (all-zero, all same sign, etc.), do not proceed to Phase 1 — re-plan instead.

### 0.4 GO

Pass = proceed to Phase 1. Fail = re-plan, no implementation.

---

## Phase 1 — `leader_dynamics.py` (AUTONOMOUS, code only)

### 1.1 New file: `src/openarm_lerobot/leader_dynamics.py`

```python
class OpenArmLeaderDynamics:
    def __init__(self, urdf_path: str, mount_rotation_xyz_deg: list[float] | None = None):
        # Pinocchio model load; rotate gravity vector by mount rpy.
        ...

    def joint4_gravity_torque(self, q_full_deg: Sequence[float]) -> float:
        # q_full_deg: 7 arm joint positions in deg (joint_1..joint_7).
        # Internal radians, output Nm.
        # Fill openarm_joint1..7 by name into Pinocchio q vector; finger joints stay 0.
        # Return tau at openarm_joint4 idx_v.
        ...
```

### 1.2 Tests: `scripts/test_leader_dynamics.py`

Assertions matching real URDF values:
- zero-pose: `abs(tau4) < 0.1`
- no-mount elbow 90°: `abs(tau4) > 1.0`
- mount rotation effect: hands-up-class poses produce different `tau4` with vs without mount rotation
- Sign **not hardcoded** — only consistency checks against Phase 0 sanity table.

### 1.3 Verify

```bash
python3 -m py_compile src/openarm_lerobot/leader_dynamics.py
python3 scripts/test_leader_dynamics.py
```

### 1.4 Commit

`feat(leader): pinocchio gravity torque for joint_4 elbow`

---

## Phase 2 — Leader GC wrappers (AUTONOMOUS, code only)

### 2.1 Edit: `src/openarm_lerobot/safe_followers.py`

Add a GC config base usable by both single-arm and nested bimanual configs (so `left_arm_config` / `right_arm_config` cleanly accept the new fields):

- `gc_motor: str = "joint_4"`
- `tau_scale: float = 0.0`
- `tau_clip_nm: float = 0.5`
- `urdf_path: str = ""`
- `mount_rotation_xyz_deg: list[float] | None = None`

Config-level guards (reject):
- `manual_control=False`
- `gc_motor != "joint_4"`
- `tau_scale < 0`
- `tau_clip_nm <= 0`

### 2.2 `SafeOpenArmLeaderGC(SafeOpenArmLeader)`

Registered as `safe_openarm_leader_gc`.

- `configure()`: `bus.disable_torque()` (all). Then if `tau_scale > 0` and dynamics initialized, `bus.enable_torque(motors=[gc_motor])` only.
- `get_action()`: parent state read first. If `tau_scale <= 0` or any `joint_1..7.pos` is `None`/non-finite, return early (no MIT command). Else compute `tau = clip(dynamics.joint4_gravity_torque(q) * tau_scale, ±tau_clip_nm)` and call `bus._mit_control_batch({gc_motor: (0.0, 0.0, q4, 0.0, tau)})`.
- `disconnect()`: always `bus.disable_torque()` in `finally` before parent disconnect.

### 2.3 `SafeBiOpenArmLeaderGC(SafeBiOpenArmLeader)`

Registered as `safe_bi_openarm_leader_gc`.

- Holds left/right `SafeOpenArmLeaderGC` instances with their own GC fields.
- `disconnect` / fault path: try-disable both arms regardless of which one errored.

### 2.4 Verify

```bash
python3 -m py_compile src/openarm_lerobot/*.py
python3 -c "from openarm_lerobot.safe_followers import SafeOpenArmLeader, SafeOpenArmLeaderGC, SafeBiOpenArmLeader, SafeBiOpenArmLeaderGC; print('ok')"
```

### 2.5 Commit

`feat(leader): SafeOpenArmLeaderGC and bimanual elbow feedforward modes`

---

## Phase 3 — Single-arm smoke (NEEDS_OPERATOR)

### 3.1 Files (AUTONOMOUS)

- Temp config: `/tmp/leader_gc_smoke_left.json` — single `safe_openarm_leader_gc`, `port=can0`, `manual_control=true`, `tau_scale=0.0`, `tau_clip_nm=0.5`, `urdf_path=assets/openarm_right.urdf`, `mount_rotation_xyz_deg=[-90.0, 0.0, 0.0]`. **Not committed.**
- New script: `scripts/test_leader_gc_singlearm.py` — leader-only `connect()`, 1 Hz loop printing `q1..q7` + raw and clipped `tau`, `finally` block forces `disable_torque`. Ctrl+C safe.

### 3.2 Live ramp (operator wait for `READY GC.<N>`)

Stop on any oscillation/runaway → Ctrl+C; script `finally` disables torque.

- **READY GC.0** — `tau_scale=0.0`. Verify behavior identical to existing freewheel (no joint_4 enable should occur).
- **READY GC.1** — `tau_scale=0.3`. Operator swings elbow freely; rates "elbow lift burden" 0=weightless..10=unchanged.
- **READY GC.2** — `tau_scale=0.7`. Same protocol. Stop here for this PR (Q2).

### 3.3 Report

`/tmp/leader_gc_tuning_${TS}.md` — per-stage operator subjective score.

### 3.4 Commit

`feat(leader): single-arm smoke harness`

---

## Phase 4 — Bimanual record validation (NEEDS_OPERATOR)

### 4.1 Files (AUTONOMOUS)

- New `configs/record_rgb_gc.json` — read-only copy of `record_rgb.json` with `teleop.type` swapped to `safe_bi_openarm_leader_gc`, and per-side GC fields (`urdf_path`, `mount_rotation_xyz_deg`, `tau_scale`, `tau_clip_nm`) under `left_arm_config` / `right_arm_config`. Default `tau_scale=0.0`.

### 4.2 Config-parse smoke (AUTONOMOUS, no robot)

```bash
OPENARM_RECORD_COMPAT_ONLY=1 \
OPENARM_RECORD_TEMPLATE_OVERRIDE=configs/record_rgb_gc.json \
  ./scripts/run_record.sh rgb compat_gc 1 5 5
```
Generated config in `.tmp/` should contain GC fields with no parse errors.

### 4.3 Live record ramp (operator wait for `READY GC.bi.<N>`)

Each stage = 1 episode, 5 s episode_time, 5 s reset_time.

- **READY GC.bi.0** — both `tau_scale=0.0`. Confirms identical behavior to baseline `record_rgb.json` recording.
- **READY GC.bi.1** — both `tau_scale=0.3`. Operator subjective score per arm.
- **READY GC.bi.2** — both `tau_scale=0.7`. Stop here.

Stop triggers (any arm): runaway, oscillation, joint-limit abort, operator discomfort. On stop, both arms must `disable_torque`.

### 4.4 Report

`/tmp/leader_gc_bimanual_${TS}.md` — per-stage scores per arm.

### 4.5 Commit

`feat(record): record_rgb_gc template for bimanual elbow GC`

---

## Out of scope (next PR)

- `tau_scale=1.0` (Q2 deferred)
- `tau_clip_nm` increase above 0.5 Nm (Q3 explicit GO required)
- joint_2 (shoulder lift) gravity comp
- Friction / stiction compensation
- `record_nocam_gc.json`, `record_full_gc.json`
- Same scheme on the right arm with empirical mount tuning if `[+90,0,0]` mirror sanity fails

---

## Implementation entry condition

When the operator opens the next session and replies `GO IMPLEMENT`, Codex should start at Phase 0 and proceed sequentially. Do not skip phases. Do not start Phase 3 / 4 live without explicit `READY GC.<N>` / `READY GC.bi.<N>`.

## Mild caveat to keep in mind during live

`tau_clip_nm=0.5` may be binding at `tau_scale=0.7` (raw elbow torque can reach ~1.5 Nm depending on pose; `1.5 × 0.7 = 1.05 Nm` → clipped to 0.5 Nm). If operator reports "still heavy" at GC.2, it is the cap, not the dynamics — escalate via Q3 (separate GO) in a follow-up PR.

---

## Output artifacts summary

| File | Type | Phase | Repo? |
|------|------|-------|-------|
| `/tmp/leader_gc_reference_summary.md` | report | 0 | no |
| `/tmp/leader_gc_sanity_${TS}.md` | report | 0 | no |
| `src/openarm_lerobot/leader_dynamics.py` | new | 1 | yes |
| `scripts/test_leader_dynamics.py` | new | 1 | yes |
| `src/openarm_lerobot/safe_followers.py` | edit | 2 | yes |
| `/tmp/leader_gc_smoke_left.json` | new | 3 | no |
| `scripts/test_leader_gc_singlearm.py` | new | 3 | yes |
| `/tmp/leader_gc_tuning_${TS}.md` | report | 3 | no |
| `configs/record_rgb_gc.json` | new | 4 | yes |
| `/tmp/leader_gc_bimanual_${TS}.md` | report | 4 | no |
