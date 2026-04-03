# RealSense RSUSB Runbook

## Current verified state

- RSUSB-built librealsense binaries work from `/home/syhlabtop/src/librealsense/build/Release`
- Verified cameras seen by RSUSB enumerate:
  - D435: `234322070493`
  - D405: `230322273311`
  - D405: `315122270766`
- Current blocker for LeRobot integration is **missing `pyrealsense2` in Python**
- `BUILD_PYTHON_BINDINGS=OFF` in `/home/syhlabtop/src/librealsense/build/CMakeCache.txt`

## Rules

1. Do not use `/usr/bin/rs-enumerate-devices` for verification.
2. Always use RSUSB build outputs from `/home/syhlabtop/src/librealsense/build/Release`.
3. Validate in this order:
   - RSUSB binary enumeration
   - Python `pyrealsense2`
   - LeRobot camera discovery
   - OpenArm dataset recording

## Phase 1 validation

### 1. Enumerate cameras with RSUSB

```bash
/home/syhlabtop/src/librealsense/build/Release/rs-enumerate-devices
```

### 2. Rebuild librealsense with Python bindings

Current build does not include `pyrealsense2`. Reconfigure the same RSUSB source tree with Python bindings enabled in the final Python environment that LeRobot will use.

Targets to verify after rebuild:

- `import pyrealsense2`
- query all 3 camera serials
- open one camera by serial
- open 3 cameras concurrently in one process

### 3. Validate LeRobot RealSense path

LeRobot relies on:

- `lerobot-find-cameras realsense`
- `RealSenseCameraConfig(serial_number_or_name=...)`

Expected camera keys for OpenArm phase 1:

- `observation.images.chest`
- `observation.images.left_wrist`
- `observation.images.right_wrist`

## Recommended first stream settings

- chest / D435: `640x480 @ 15fps`, color only
- wrist / D405: `640x480 @ 15fps`, color + optional depth

If 3-camera stability is poor, reduce in this order:

1. disable depth on wrist cameras
2. keep all cameras at 15fps
3. lower chest resolution further

## Failure classification

### Python binding problem

- `import pyrealsense2` fails
- standalone Python cannot enumerate cameras

### LeRobot config problem

- standalone `pyrealsense2` works
- `lerobot-find-cameras realsense` fails or sees fewer cameras

### Bandwidth / topology problem

- single-camera tests pass
- 3-camera concurrent open causes drops, resets, or missing streams

### Dataset writer problem

- cameras are stable until recording/encoding starts

## OpenArm phase-1 target

Use `bi_openarm_follower` with three serial-based RealSense camera configs and record a short local dataset before moving to larger episode counts.
