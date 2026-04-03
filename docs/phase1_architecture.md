# Phase 1 Architecture

## Objective

Build the first OpenArm data acquisition pipeline on top of working bilateral teleoperation.

## Data Flow

1. `openarm_teleop` produces leader/follower state and follower references.
2. `openarm_lerobot` records synchronized action/observation/time tuples.
3. Episodes are written in a LeRobot-compatible schema.
4. Later, camera streams are added and synchronized to the same episode timeline.

## Phase 1 Observation / Action Contract

- Observation:
  - follower arm joint positions
  - follower arm joint velocities
  - follower gripper state
  - optional follower efforts if cheap to record
- Action:
  - follower arm joint references
  - follower gripper reference
- Auxiliary only:
  - leader state
  - operator/task metadata
  - intervention and success flags

## Baseline Order

1. Proprio-only behavior cloning
2. Single-camera ACT
3. Multi-camera ACT / diffusion after pipeline stabilizes

## Why Cameras Matter

Embodiment-mismatched public pretrained policies are unlikely to transfer cleanly.
Camera streams are needed for realistic LeRobot visuomotor baselines and future VLA work.
