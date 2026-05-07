# `configs` KNOWLEDGE BASE

## OVERVIEW

Editable source-of-truth for recording presets and RealSense camera mapping.

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| No-camera baseline | `record_nocam.json` | Proprio-only recording preset |
| RGB camera recording | `record_rgb.json` | Chest + wrist RGB streams |
| Depth-enabled recording | `record_full.json` | Wrist depth enabled |
| Fixture/compat config | `record_rgb_head_fixture.json` | Camera-key substitution fixture |
| Quest left-arm recording | `record_quest_left_nocam.json` | Left Quest controller to physical left arm on `can0`; legacy calibration id retained |
| Quest right-arm recording | `record_quest_right_nocam.json` | Right Quest controller to physical right arm on `can1`; calibration/no-send validation required |
| RealSense mapping | `realsense_3cam_mapping.yaml` | Serials, ports, RSUSB notes |

## CONVENTIONS

- Record preset shape is top-level `robot`, `teleop`, `dataset`, plus `display_data`, `play_sounds`, `resume`.
- Camera keys should converge on `left_wrist`, `right_wrist`, `chest`; `wrist_a`/`wrist_b` are transitional mapping names.
- Dataset defaults use local IDs like `local/<run_name>` and local roots under `data/<run_name>`.
- `record_quest_right_nocam.json` is a different schema from bimanual camera presets: `safe_openarm_follower` + `quest_spatial_teleop`.
- Verified bus-to-arm map from 2026-05-08 LED ENABLE test: `can0` is the physical left arm, `can1` is the physical right arm. Both arms use motor IDs `0x01..0x08`; do not change motor IDs to swap arms.
- Final controller target is left Quest controller -> left arm and right Quest controller -> right arm. Earlier tuning used the right Quest controller against the left arm during bus-map discovery, so both final paths need no-send axis validation before live.
- Keep joint order, motor IDs, KP/KD arrays, and joint limits aligned in the Quest config.

## ANTI-PATTERNS

- Do not change physical camera serials or ports without updating docs/runbooks and validator expectations.
- Do not assume `dataset.fps` changes per-camera FPS; set camera FPS under each camera config.
- Do not enable Hub upload by default; use wrapper env overrides for explicit upload.
- Do not hand-edit generated `.tmp/record_*` configs as source; edit these presets instead.

## COMMANDS

```bash
./scripts/run_record.sh nocam smoke 1 20 20
./scripts/run_record.sh rgb stable20 2 20 20 --dataset.fps=20 --robot.left_arm_config.cameras.chest.fps=20
python3 scripts/check_lerobot_recording_compatibility.py --preset rgb --run-name compat
```

## NOTES

- RSUSB note: use the locally built librealsense binaries/libraries, not apt-installed librealsense tools.
- Camera stability path: start `640x480 @ 30fps`, then try `20fps`, then `15fps`; disable wrist depth before broader downgrades if needed.
