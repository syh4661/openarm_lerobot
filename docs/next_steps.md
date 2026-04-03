# Immediate Next Steps

## Before LeRobot recording

1. Install Python 3.12
2. Install `ffmpeg`
3. Rebuild librealsense with `BUILD_PYTHON_BINDINGS=ON`
4. Ensure `pyrealsense2` imports from the same Python environment used by LeRobot

### Current Ubuntu 22.04 bootstrap on this machine

LeRobot requires Python `>=3.12`. The temporary Python 3.10 path can validate RSUSB camera access, but it cannot install LeRobot itself.

Run:

```bash
sudo /home/syhlabtop/workspace/openarm_lerobot/scripts/install_python312_for_lerobot_ubuntu22.sh
/home/syhlabtop/workspace/openarm_lerobot/scripts/rebuild_pyrealsense2_py312.sh
/home/syhlabtop/workspace/openarm_lerobot/scripts/bootstrap_lerobot_runtime_py312.sh
```

Then activate:

```bash
source /home/syhlabtop/workspace/openarm_lerobot/.venv312/bin/activate
source /home/syhlabtop/workspace/openarm_lerobot/scripts/env_rsusb_py312.sh
```

## Before final camera naming

1. Mount chest D435
2. Mount two wrist D405 cameras
3. Physically identify which D405 is left vs right
4. Rename `wrist_a` / `wrist_b` mapping to `left_wrist` / `right_wrist`

## Before full OpenArm data collection

1. Single-camera Python smoke test
2. Three-camera Python smoke test
3. `lerobot-find-cameras realsense`
4. Short LeRobot camera-only smoke test
5. Short OpenArm + camera record test
