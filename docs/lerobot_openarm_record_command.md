# OpenArm + LeRobot record command draft

## Current status

- RealSense RSUSB runtime works
- `pyrealsense2` works in Python when using the RSUSB build path
- LeRobot source is available locally
- LeRobot runtime must use **Python 3.12+**; Python 3.10 validation is useful for RSUSB camera checks only

## Required runtime environment

Before running any LeRobot command in the final local setup:

```bash
source /home/syhlabtop/workspace/openarm_lerobot/.venv312/bin/activate
source /home/syhlabtop/workspace/openarm_lerobot/scripts/env_rsusb_py312.sh
```

## Intended camera mapping

- `left_wrist`  -> D405 serial `315122270766`
- `right_wrist` -> D405 serial `230322273311`
- `chest`       -> D435 serial `234322070493`

## Draft record command

This is the target command shape after LeRobot dependencies are installed:

```bash
source /home/syhlabtop/workspace/openarm_lerobot/.venv312/bin/activate
source /home/syhlabtop/workspace/openarm_lerobot/scripts/env_rsusb_py312.sh

python3 -m lerobot.scripts.lerobot_record \
  --robot.type=bi_openarm_follower \
  --robot.id=openarm_bimanual_follower \
  --robot.left_arm_config.port=can3 \
  --robot.left_arm_config.side=left \
  --robot.left_arm_config.cameras='{
left_wrist: {type: intelrealsense, serial_number_or_name: "315122270766", width: 640, height: 480, fps: 15, use_depth: true},
    chest: {type: intelrealsense, serial_number_or_name: "234322070493", width: 640, height: 480, fps: 15, use_depth: false}
  }' \
  --robot.right_arm_config.port=can2 \
  --robot.right_arm_config.side=right \
  --robot.right_arm_config.cameras='{
right_wrist: {type: intelrealsense, serial_number_or_name: "230322273311", width: 640, height: 480, fps: 15, use_depth: true}
  }' \
  --teleop.type=bi_openarm_leader \
  --teleop.left_arm_config.port=can1 \
  --teleop.right_arm_config.port=can0 \
  --teleop.id=openarm_bimanual_leader \
  --dataset.repo_id=<hf_user>/openarm_phase1 \
  --dataset.single_task="OpenArm teleoperation phase-1 collection" \
  --dataset.root=/home/syhlabtop/workspace/openarm_lerobot/data/openarm_phase1 \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=20 \
  --dataset.fps=15 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --display_data=true
```

## Notes

- Replace the left/right wrist serial assignment after physical mounting is finalized.
- Keep all cameras at `640x480 @ 15fps` for the first full-system smoke test.
- If stability is poor, disable depth on wrist cameras before reducing other settings.
- This command should only be used after LeRobot dependencies are installed and `lerobot-find-cameras realsense` succeeds.
