# Quest Isaac Real Control Contract

This contract freezes the Quest spatial replay shape used for Isaac and real hardware promotion.

## Canonical Quest spatial action schema

The teleop emits only these fields:

| Field | Unit | Meaning |
| --- | --- | --- |
| `quest.pos_delta.x` | m | Calibrated translation delta along X |
| `quest.pos_delta.y` | m | Calibrated translation delta along Y |
| `quest.pos_delta.z` | m | Calibrated translation delta along Z |
| `quest.rot_delta.rx` | rad | Calibrated rotation vector X |
| `quest.rot_delta.ry` | rad | Calibrated rotation vector Y |
| `quest.rot_delta.rz` | rad | Calibrated rotation vector Z |
| `quest.gripper` | 0 to 1 | Normalized trigger command, where 0 is closed and 1 is open within the frozen gripper range |
| `quest.enabled` | 0 or 1 | Deadman gate, 1 means active tracking, 0 means hold or stop |

Frozen source behavior:

- right controller only for the single arm replay path
- USB only, no network Quest input
- `quest.enabled = 0` when the grip is not held, the controller disappears, or the delta cannot be computed
- disabled or unavailable input produces a zero spatial action with neutral gripper command `0.5`

## Frozen frame chain

`Quest controller frame -> Quest calibration reference -> operator/world frame -> OpenArm base frame -> openarm_hand_tcp`

This contract keeps the canonical downstream target at `openarm_hand_tcp` and does not redefine the OpenArm chain.

## Frozen OpenArm joint and gripper contract

Per arm, the seven joint chain is fixed as:

- `openarm_joint1`
- `openarm_joint2`
- `openarm_joint3`
- `openarm_joint4`
- `openarm_joint5`
- `openarm_joint6`
- `openarm_joint7`

Per arm motor order is fixed as:

- `joint_1`
- `joint_2`
- `joint_3`
- `joint_4`
- `joint_5`
- `joint_6`
- `joint_7`
- `gripper`

Frozen gripper semantics:

- hardware range is `[-65, 0]` degrees
- Quest trigger is normalized into `quest.gripper`
- `MapQuestActionToRobotAction` converts the normalized command into `gripper_vel` with neutral `0.5`

Frozen end effector step limit:

- `max_ee_step_m = 0.05`

## Downstream processor contract

`QuestSpatialTeleop` emits the frozen Quest fields above.
`MapQuestActionToRobotAction` consumes them and emits the robot action keys:

- `enabled`
- `target_x`
- `target_y`
- `target_z`
- `target_wx`
- `target_wy`
- `target_wz`
- `gripper_vel`

When `quest.enabled = 0`, all target components and `gripper_vel` must be zeroed.

## Bimanual semantics

Live bimanual ownership is frozen as:

- left Quest controller drives left arm
- right Quest controller drives right arm

Replay bimanual must pass before any live bimanual command is allowed.

Each arm keeps the same frozen seven joint chain, motor order, frame target, and gripper semantics.

## Promotion thresholds

Promotion from replay into Isaac and then real hardware requires all of these thresholds:

- `max_tracking_error_m <= 0.03`
- `control_rate_hz >= 25`
- `collision_count == 0`
- `nan_count == 0`
- `disabled_drift_m <= 0.005`
- `stop_latency_s <= 0.5`

## Evidence JSON schema

Every replay and gate report must be machine parseable and include these top level keys:

- `schema_version`
- `gate_name`
- `trace_file`
- `status`
- `control_rate_hz`
- `metrics`
- `thresholds`
- `samples`

Required report rules:

- `schema_version` is an integer.
- `status` is `pass` or `fail`.
- `metrics` is an object that records the gate measurements.
- `thresholds` repeats the frozen promotion thresholds above so later gates can compare results without guessing.
- `samples` is a deterministic array of replay points.

Each sample should carry:

- `t_s`
- `arm`
- the eight frozen Quest fields
- `tracking_state`
- `expected_behavior`

## Deterministic replay rules

- fixtures stay small and offline friendly
- fixtures use the spatial Quest schema directly
- fixtures must not depend on Quest hardware, Isaac, or real robot motion
- replay data must stay deterministic so the later gate scripts can compare exact values
