# `src/openarm_lerobot` KNOWLEDGE BASE

## OVERVIEW

Safety-sensitive Quest/OpenArm adapter package: teleop state, IK/action mapping, safe robot wrappers, remote bridge, and NumPy msgpack helpers.

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Quest right-arm teleop | `quest_teleop.py` | Main hotspot; calibration, IK, hold-last-action |
| Spatial action pipeline | `quest_spatial_teleop.py`, `quest_processor.py` | Quest fields → robot action schema |
| Quest reader lifecycle | `quest_reader.py` | Hardware side effects deferred until `connect()` |
| Safe OpenArm wrappers | `safe_followers.py` | Torque disable, calibration namespace reuse |
| Policy bridge | `bridge_client.py`, `msgpack_numpy.py` | WebSocket + msgpack NumPy serialization |
| Public API | `__init__.py` | Small lazy re-export facade |

## CONVENTIONS

- Keep imports resilient around missing LeRobot/Quest symbols where current modules already use lazy resolution or fallback classes.
- Preserve frozen hardware constants unless the physical robot contract changes: controller side, URDF target frame, joint order, motor names, gripper range.
- Use module-level helpers for validation/coercion; do not hide shape/finite checks inside call sites.
- Teleop classes follow LeRobot interface methods: `action_features`, `feedback_features`, `connect`, `calibrate`, `get_action`, `disconnect`.
- `QUEST_DEBUG` is the opt-in debug surface; avoid always-on high-rate logs in control loops.

## ANTI-PATTERNS

- Do not bypass `_validate_frozen_contract` / `_validate_spatial_contract` when adding config fields.
- Do not remove translation clipping or hold-last-action fallbacks from Quest teleop.
- Do not make orientation tracking live without a hardware-safety review; current main teleop intentionally freezes orientation.
- Do not replace `msgpack_numpy.py` with pickle-like serialization for policy bridge traffic.
- Do not assume LSP import diagnostics are conclusive unless the sibling LeRobot/OpenArm runtime is active.

## HOTSPOTS

- `quest_teleop.py`: state flow `disconnected → connected_uncalibrated → calibrated_idle → tracking`; failures return idle/hold actions.
- `safe_followers.py`: `_safe_disable_all_motors` retries CAN torque disable and aggregates failures; treat errors as real shutdown failures.
- `quest_processor.py`: feature names must stay aligned with downstream LeRobot dataset/action keys.

## VALIDATION

```bash
python3 scripts/test_quest_processor_steps.py
python3 scripts/test_quest_ik_roundtrip.py --urdf assets/openarm_right.urdf --target-frame openarm_hand_tcp
python3 -m py_compile src/openarm_lerobot/*.py
```
