#!/usr/bin/env python3

from __future__ import annotations

import argparse
from importlib import import_module
import json
import logging
import os
from pathlib import Path
import sys
import time
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
for source_path in (REPO_ROOT / "src", REPO_ROOT.parent / "lerobot" / "src"):
    if source_path.exists():
        sys.path.insert(0, str(source_path))
if (REPO_ROOT.parent / "openarm_description" / "package.xml").exists():
    ros_package_paths = [
        path for path in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep) if path
    ]
    workspace_path = str(REPO_ROOT.parent)
    if workspace_path not in ros_package_paths:
        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(
            [workspace_path, *ros_package_paths]
        )


feature_utils_module = import_module("lerobot.datasets.feature_utils")
dataset_module = import_module("lerobot.datasets.lerobot_dataset")
pipeline_features_module = import_module("lerobot.datasets.pipeline_features")
processor_module = import_module("lerobot.processor")
converters_module = import_module("lerobot.processor.converters")
robot_kinematic_processor_module = import_module(
    "lerobot.robots.so_follower.robot_kinematic_processor"
)
record_module = import_module("lerobot.scripts.lerobot_record")
openarm_kinematics_module = import_module("openarm_lerobot.kinematics")
quest_processor_module = import_module("openarm_lerobot.quest_processor")
quest_spatial_teleop_module = import_module("openarm_lerobot.quest_spatial_teleop")
quest_teleop_module = import_module("openarm_lerobot.quest_teleop")
safe_followers_module = import_module("openarm_lerobot.safe_followers")
operator_notify_module = import_module("openarm_lerobot.operator_notify")

combine_feature_dicts = getattr(feature_utils_module, "combine_feature_dicts")
LeRobotDataset = getattr(dataset_module, "LeRobotDataset")
aggregate_pipeline_dataset_features = getattr(
    pipeline_features_module, "aggregate_pipeline_dataset_features"
)
create_initial_features = getattr(pipeline_features_module, "create_initial_features")
OpenArmKinematics = getattr(openarm_kinematics_module, "OpenArmKinematics")
RobotProcessorPipeline = getattr(processor_module, "RobotProcessorPipeline")
observation_to_transition = getattr(converters_module, "observation_to_transition")
robot_action_observation_to_transition = getattr(
    converters_module, "robot_action_observation_to_transition"
)
transition_to_observation = getattr(converters_module, "transition_to_observation")
transition_to_robot_action = getattr(converters_module, "transition_to_robot_action")
EEBoundsAndSafety = getattr(robot_kinematic_processor_module, "EEBoundsAndSafety")
EEReferenceAndDelta = getattr(robot_kinematic_processor_module, "EEReferenceAndDelta")
ForwardKinematicsJointsToEE = getattr(
    robot_kinematic_processor_module, "ForwardKinematicsJointsToEE"
)
GripperVelocityToJoint = getattr(
    robot_kinematic_processor_module, "GripperVelocityToJoint"
)
InverseKinematicsEEToJoints = getattr(
    robot_kinematic_processor_module, "InverseKinematicsEEToJoints"
)
record_loop = getattr(record_module, "record_loop")
MapQuestActionToRobotAction = getattr(
    quest_processor_module, "MapQuestActionToRobotAction"
)
OpenArmGripperVelocityToJoint = getattr(
    quest_processor_module, "OpenArmGripperVelocityToJoint"
)
QuestSpatialTeleop = getattr(quest_spatial_teleop_module, "QuestSpatialTeleop")
QuestSpatialTeleopConfig = getattr(
    quest_spatial_teleop_module, "QuestSpatialTeleopConfig"
)
QUEST_OPENARM_MOTOR_NAMES = getattr(quest_teleop_module, "QUEST_OPENARM_MOTOR_NAMES")
QUEST_OPENARM_TARGET_FRAME = getattr(quest_teleop_module, "QUEST_OPENARM_TARGET_FRAME")
QUEST_OPENARM_URDF_JOINT_NAMES = getattr(
    quest_teleop_module, "QUEST_OPENARM_URDF_JOINT_NAMES"
)
_log_quest_debug = getattr(quest_teleop_module, "_log_quest_debug")
SafeOpenArmFollower = getattr(safe_followers_module, "SafeOpenArmFollower")
SafeOpenArmFollowerConfig = getattr(safe_followers_module, "SafeOpenArmFollowerConfig")
notify = getattr(operator_notify_module, "notify")
confirm = getattr(operator_notify_module, "confirm")


logger = logging.getLogger(__name__)

_QUEST_ENABLED_HOLD_KEY = "_quest.enabled_for_hold"
_QUEST_ZERO_DELTA_HOLD_KEY = "_quest.zero_delta_hold"
_ZERO_DELTA_EPS = 1e-9


class CurrentHoldEEReferenceAndDelta(EEReferenceAndDelta):
    """Update the disabled EE hold target from current FK each frame."""

    def action(self, action: Any) -> Any:
        try:
            enabled = bool(action.get("enabled", False))
        except (TypeError, ValueError):
            enabled = False
        zero_delta = enabled and _action_has_zero_ee_delta(action)
        if not enabled:
            self._command_when_disabled = None
        output = super().action(action)
        output[_QUEST_ENABLED_HOLD_KEY] = 1.0 if enabled else 0.0
        output[_QUEST_ZERO_DELTA_HOLD_KEY] = 1.0 if zero_delta else 0.0
        return output


class CurrentAwareEEBoundsAndSafety(EEBoundsAndSafety):
    """Avoid EE jump checks when Quest tracking is explicitly disabled."""

    def action(self, action: Any) -> Any:
        try:
            enabled = bool(float(action.get(_QUEST_ENABLED_HOLD_KEY, 0.0)))
        except (TypeError, ValueError):
            enabled = False
        if not enabled:
            self._last_pos = None
        return super().action(action)


class NoSendActionRobot:
    def __init__(self, robot: Any):
        self._robot = robot

    def __getattr__(self, name: str) -> Any:
        return getattr(self._robot, name)

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        logger.info(
            "No-send mode active; skipping robot.send_action with keys=%s",
            sorted(action.keys()),
        )
        return action


class QuestDebugRobotActionProcessor:
    def __init__(self, processor: Any):
        self._processor = processor

    def __getattr__(self, name: str) -> Any:
        return getattr(self._processor, name)

    def __call__(self, data: Any) -> Any:
        quest_gripper = None
        quest_enabled = None
        observed_gripper = None
        if isinstance(data, tuple) and len(data) == 2:
            raw_action, observation = data
            if isinstance(raw_action, dict):
                quest_gripper = raw_action.get("quest.gripper")
                quest_enabled = raw_action.get("quest.enabled")
            if isinstance(observation, dict):
                observed_gripper = observation.get("gripper.pos")
        output = self._processor(data)
        commanded_gripper = output.get("gripper.pos") if isinstance(output, dict) else None
        gripper_vel = None
        gripper_delta = None
        commanded_gripper_delta = None
        try:
            enabled = bool(float(quest_enabled)) if quest_enabled is not None else False
            normalized = float(quest_gripper)
            gripper_vel = (normalized - 0.5) * 2.0 if enabled else 0.0
            gripper_delta = gripper_vel * 10.0
        except (TypeError, ValueError):
            pass
        try:
            if observed_gripper is not None and commanded_gripper is not None:
                commanded_gripper_delta = float(commanded_gripper) - float(
                    observed_gripper
                )
        except (TypeError, ValueError):
            pass
        _log_quest_debug(
            event="closed_loop_gripper_command",
            quest_enabled=quest_enabled,
            quest_gripper=quest_gripper,
            gripper_vel=gripper_vel,
            gripper_delta_deg=gripper_delta,
            observed_gripper_pos=observed_gripper,
            commanded_gripper_pos=commanded_gripper,
            commanded_gripper_delta_deg=commanded_gripper_delta,
        )
        _log_quest_debug(
            event="closed_loop_joint_command",
            commanded_joint_angles_deg=_ordered_joint_positions(output),
        )
        return output


class HoldWhenQuestDisabledRobotActionProcessor:
    """Bypass IK when Quest cannot produce a meaningful EE motion command."""

    def __init__(self, processor: Any, joint_limits: dict[str, Any] | None = None):
        self._processor = processor
        self._joint_limits = joint_limits or {}
        self._latched_hold_action: dict[str, float] | None = None

    def __getattr__(self, name: str) -> Any:
        return getattr(self._processor, name)

    def __call__(self, data: Any) -> dict[str, float]:
        if not isinstance(data, tuple) or len(data) != 2:
            return self._processor(data)

        raw_action, observation = data
        if not isinstance(raw_action, dict) or not isinstance(observation, dict):
            return self._processor(data)

        action = dict(raw_action)
        marker = action.pop(_QUEST_ENABLED_HOLD_KEY, 0.0)
        zero_delta_marker = action.pop(_QUEST_ZERO_DELTA_HOLD_KEY, 0.0)
        try:
            quest_enabled = bool(float(marker))
        except (TypeError, ValueError):
            quest_enabled = False
        try:
            zero_delta_hold = bool(float(zero_delta_marker))
        except (TypeError, ValueError):
            zero_delta_hold = False

        if quest_enabled:
            if not zero_delta_hold:
                self._latched_hold_action = None
                return self._processor((action, observation))

            output = _hold_observed_motor_positions(
                observation,
                self._joint_limits,
                gripper_pos=action.get("ee.gripper_pos"),
            )
            _log_quest_debug(
                event="closed_loop_zero_delta_joint_hold",
                commanded_joint_angles_deg=_ordered_joint_positions(output),
            )
            return output

        if self._latched_hold_action is None:
            self._latched_hold_action = _hold_observed_motor_positions(
                observation, self._joint_limits
            )
        output = dict(self._latched_hold_action)
        _log_quest_debug(
            event="closed_loop_disabled_joint_hold",
            commanded_joint_angles_deg=_ordered_joint_positions(output),
        )
        return output


def _hold_observed_motor_positions(
    observation: dict[str, Any],
    joint_limits: dict[str, Any],
    *,
    gripper_pos: Any | None = None,
) -> dict[str, float]:
    output: dict[str, float] = {}
    for motor_name in QUEST_OPENARM_MOTOR_NAMES:
        key = f"{motor_name}.pos"
        if key not in observation:
            raise ValueError(f"Observation missing {key!r} for disabled joint hold.")
        if motor_name == "gripper" and gripper_pos is not None:
            value = float(gripper_pos)
        else:
            value = float(observation[key])
        if motor_name in joint_limits:
            lower, upper = joint_limits[motor_name]
            value = min(max(value, float(lower)), float(upper))
        output[key] = value
    return output


def _action_has_zero_ee_delta(action: dict[str, Any]) -> bool:
    for key in (
        "target_x",
        "target_y",
        "target_z",
        "target_wx",
        "target_wy",
        "target_wz",
    ):
        try:
            if abs(float(action.get(key, 0.0))) > _ZERO_DELTA_EPS:
                return False
        except (TypeError, ValueError):
            return False
    return True


def _ordered_joint_positions(action: dict[str, Any]) -> list[float | None]:
    values: list[float | None] = []
    for motor_name in QUEST_OPENARM_MOTOR_NAMES:
        raw = action.get(f"{motor_name}.pos")
        try:
            values.append(float(raw))
        except (TypeError, ValueError):
            values.append(None)
    return values


def _disconnect_if_connected(device: Any, *, label: str) -> None:
    if not bool(getattr(device, "is_connected", False)):
        return
    try:
        device.disconnect()
    except Exception:
        logger.exception("Failed to disconnect %s cleanly.", label)


def _motor_pos_key(name: str) -> str:
    return f"{name}.pos"


def _joint_limit_clamp(robot: Any, motor_name: str, value: float) -> float:
    joint_limits = getattr(getattr(robot, "config", None), "joint_limits", {})
    if motor_name not in joint_limits:
        return float(value)
    lower, upper = joint_limits[motor_name]
    return min(max(float(value), float(lower)), float(upper))


def _joint_limit_bounds(robot: Any, motor_name: str) -> tuple[float, float] | None:
    joint_limits = getattr(getattr(robot, "config", None), "joint_limits", {})
    if motor_name not in joint_limits:
        return None
    lower, upper = joint_limits[motor_name]
    return float(lower), float(upper)


def _read_motor_positions(robot: Any) -> dict[str, float]:
    observation = robot.get_observation()
    positions: dict[str, float] = {}
    for name in [f"joint_{index}" for index in range(1, 8)] + ["gripper"]:
        key = _motor_pos_key(name)
        if key in observation:
            positions[name] = float(observation[key])
    return positions


def ramp_to_init_pose(
    robot: Any,
    target_pose_deg: dict[str, float],
    max_step_deg: float,
    timeout_s: float,
    *,
    tick_hz: float = 30.0,
    max_observed_lag_deg: float | None = 5.0,
) -> None:
    """Slowly ramp robot joints to target pose using send_action."""

    if max_step_deg <= 0.0:
        raise ValueError("init_pose_max_step_deg must be positive.")
    if timeout_s <= 0.0:
        raise ValueError("init_pose_timeout_s must be positive.")
    if tick_hz <= 0.0:
        raise ValueError("tick_hz must be positive.")
    if max_observed_lag_deg is not None and max_observed_lag_deg <= 0.0:
        raise ValueError("init_pose_max_observed_lag_deg must be positive.")

    motor_names = [f"joint_{index}" for index in range(1, 8)]
    target: dict[str, float] = {}
    for name in motor_names:
        requested_target = float(target_pose_deg.get(name, 0.0))
        bounds = _joint_limit_bounds(robot, name)
        if bounds is not None:
            lower, upper = bounds
            if requested_target < lower or requested_target > upper:
                raise ValueError(
                    f"init_pose_deg {name}={requested_target:.3f} deg is outside "
                    f"joint limits [{lower:.3f}, {upper:.3f}] deg."
                )
        target[name] = requested_target
    positions = _read_motor_positions(robot)
    command_pos = {
        name: _joint_limit_clamp(robot, name, positions.get(name, 0.0))
        for name in motor_names
    }
    gripper_hold = (
        _joint_limit_clamp(robot, "gripper", positions["gripper"])
        if "gripper" in positions
        else None
    )
    deadline = time.monotonic() + float(timeout_s)
    period_s = 1.0 / tick_hz

    while True:
        current = _read_motor_positions(robot)
        action: dict[str, float] = {}
        max_command_error = 0.0
        max_observed_error = 0.0
        max_observed_lag = 0.0
        for name in motor_names:
            target_pos = target[name]
            command_error = target_pos - command_pos[name]
            observed_pos = _joint_limit_clamp(robot, name, current.get(name, 0.0))
            observed_error = target_pos - observed_pos
            observed_lag = command_pos[name] - observed_pos
            max_command_error = max(max_command_error, abs(command_error))
            max_observed_error = max(max_observed_error, abs(observed_error))
            max_observed_lag = max(max_observed_lag, abs(observed_lag))
            if abs(command_error) <= 0.3:
                next_pos = target_pos
            else:
                step = min(abs(command_error), float(max_step_deg))
                next_pos = command_pos[name] + (
                    step if command_error > 0.0 else -step
                )
            command_pos[name] = _joint_limit_clamp(robot, name, next_pos)
            action[_motor_pos_key(name)] = _joint_limit_clamp(robot, name, next_pos)

        if gripper_hold is not None:
            action[_motor_pos_key("gripper")] = gripper_hold

        _log_quest_debug(
            event="init_pose_ramp",
            max_command_error_deg=max_command_error,
            max_observed_error_deg=max_observed_error,
            max_observed_lag_deg=max_observed_lag,
            target_pose_deg=target,
            commanded_action=action,
        )
        if (
            max_observed_lag_deg is not None
            and max_observed_lag > max_observed_lag_deg
        ):
            raise RuntimeError(
                "Init pose ramp observed lag exceeded "
                f"{max_observed_lag_deg:.3f} deg; "
                f"max_observed_lag={max_observed_lag:.3f} deg."
            )
        robot.send_action(action)

        if max_command_error <= 0.3:
            _log_quest_debug(
                event="init_pose_ramp_complete",
                max_command_error_deg=max_command_error,
                max_observed_error_deg=max_observed_error,
                max_observed_lag_deg=max_observed_lag,
                target_pose_deg=target,
            )
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"Init pose ramp timed out after {timeout_s}s; "
                f"max_command_error={max_command_error:.3f} deg, "
                f"max_observed_error={max_observed_error:.3f} deg."
            )
        time.sleep(period_s)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Quest closed-loop recorder entrypoint."
    )
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to JSON record config."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run a short control loop for validation.",
    )
    parser.add_argument(
        "--no-send-action",
        action="store_true",
        help="Execute the processor pipeline without commanding the robot.",
    )
    parser.add_argument(
        "--control-time-s",
        type=int,
        default=None,
        help="Override record loop duration in seconds.",
    )
    return parser.parse_args()


def load_raw_config(config_path: Path) -> dict[str, Any]:
    return json.loads(config_path.read_text())


def make_robot_config(raw: dict[str, Any]) -> Any:
    robot_raw = dict(raw["robot"])
    robot_raw.pop("type", None)
    return SafeOpenArmFollowerConfig(**robot_raw)


def make_teleop_config(raw: dict[str, Any]) -> Any:
    teleop_raw = dict(raw["teleop"])
    teleop_raw.pop("type", None)
    teleop_raw.pop("initial_joint_seed_deg", None)
    teleop_raw.pop("motor_names", None)
    teleop_raw.pop("joint_offsets_deg", None)
    teleop_raw.pop("urdf_path", None)
    teleop_raw.pop("gripper_speed_factor", None)
    teleop_raw.pop("gripper_clip_min", None)
    teleop_raw.pop("gripper_clip_max", None)
    teleop_raw.pop("gripper_invert_velocity", None)
    teleop_raw.pop("gripper_control_mode", None)
    teleop_raw.pop("gripper_max_step_deg", None)
    return QuestSpatialTeleopConfig(**teleop_raw)


def make_kinematics(urdf_path: Path) -> Any:
    return OpenArmKinematics(
        urdf_path=str(urdf_path),
        target_frame_name=QUEST_OPENARM_TARGET_FRAME,
        joint_names=list(QUEST_OPENARM_URDF_JOINT_NAMES),
        posture_weight=0.005,
    )


def _gripper_processor_kwargs(raw: dict[str, Any]) -> dict[str, float]:
    teleop_raw = raw.get("teleop", {})
    robot_limits = raw.get("robot", {}).get("joint_limits", {})
    gripper_limits = robot_limits.get("gripper", [-65.0, 0.0])
    return {
        "speed_factor": float(teleop_raw.get("gripper_speed_factor", 10.0)),
        "clip_min": float(teleop_raw.get("gripper_clip_min", gripper_limits[0])),
        "clip_max": float(teleop_raw.get("gripper_clip_max", gripper_limits[1])),
    }


def make_processors(raw: dict[str, Any], kinematics: Any) -> tuple[Any, Any, Any]:
    motor_names = list(QUEST_OPENARM_MOTOR_NAMES)
    teleop_raw = raw.get("teleop", {})
    gripper_invert_velocity = bool(
        teleop_raw.get("gripper_invert_velocity", False)
    )
    gripper_scale = -2.0 if gripper_invert_velocity else 2.0
    gripper_kwargs = _gripper_processor_kwargs(raw)
    gripper_control_mode = str(teleop_raw.get("gripper_control_mode", "velocity"))
    gripper_processor = GripperVelocityToJoint(**gripper_kwargs)
    if gripper_control_mode == "hold_ramp":
        gripper_processor = OpenArmGripperVelocityToJoint(
            clip_min=gripper_kwargs["clip_min"],
            clip_max=gripper_kwargs["clip_max"],
            max_step_deg=float(teleop_raw.get("gripper_max_step_deg", 0.5)),
        )
    elif gripper_control_mode != "velocity":
        raise ValueError(
            "teleop.gripper_control_mode must be either 'velocity' or 'hold_ramp'."
        )
    logger.info(
        "Quest gripper processor config: mode=%s scale=%s speed_factor=%s clip=[%s, %s]",
        gripper_control_mode,
        gripper_scale,
        gripper_kwargs["speed_factor"],
        gripper_kwargs["clip_min"],
        gripper_kwargs["clip_max"],
    )
    teleop_action_processor = RobotProcessorPipeline(
        steps=[
            MapQuestActionToRobotAction(gripper_scale=gripper_scale),
            CurrentHoldEEReferenceAndDelta(
                kinematics=kinematics,
                end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            CurrentAwareEEBoundsAndSafety(
                end_effector_bounds={"min": [-2.0, -2.0, -2.0], "max": [2.0, 2.0, 2.0]},
                max_ee_step_m=0.05,
            ),
            gripper_processor,
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
    robot_action_processor = RobotProcessorPipeline(
        steps=[
            InverseKinematicsEEToJoints(
                kinematics=kinematics,
                motor_names=motor_names,
                initial_guess_current_joints=True,
            )
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
    robot_action_processor = HoldWhenQuestDisabledRobotActionProcessor(
        robot_action_processor,
        joint_limits=dict(raw.get("robot", {}).get("joint_limits", {})),
    )
    robot_observation_processor = RobotProcessorPipeline(
        steps=[
            ForwardKinematicsJointsToEE(kinematics=kinematics, motor_names=motor_names)
        ],
        to_transition=observation_to_transition,
        to_output=transition_to_observation,
    )
    return teleop_action_processor, robot_action_processor, robot_observation_processor


def make_dataset(
    raw: dict[str, Any],
    robot: Any,
    teleop: Any,
    teleop_action_processor: Any,
    robot_observation_processor: Any,
) -> Any:
    dataset_raw = raw["dataset"]
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=teleop.action_features),
            use_videos=bool(dataset_raw["video"]),
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(
                observation=robot.observation_features
            ),
            use_videos=bool(dataset_raw["video"]),
        ),
    )
    return LeRobotDataset.create(
        repo_id=dataset_raw["repo_id"],
        root=dataset_raw["root"],
        fps=int(dataset_raw["fps"]),
        features=features,
        robot_type=robot.name,
        use_videos=bool(dataset_raw["video"]),
        image_writer_threads=int(dataset_raw.get("encoder_threads", 2)),
    )


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        force=True,
    )

    raw = load_raw_config(args.config)
    robot_config = make_robot_config(raw)
    teleop_config = make_teleop_config(raw)
    robot = SafeOpenArmFollower(robot_config)
    teleop = QuestSpatialTeleop(teleop_config)
    kinematics = make_kinematics(
        Path(raw["teleop"].get("urdf_path", "assets/openarm_right.urdf"))
    )
    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_processors(raw, kinematics)
    )
    robot_action_processor = QuestDebugRobotActionProcessor(robot_action_processor)
    dataset = make_dataset(
        raw, robot, teleop, teleop_action_processor, robot_observation_processor
    )
    events = {"exit_early": False, "stop_recording": False, "rerecord_episode": False}
    control_time_s = args.control_time_s
    if control_time_s is None:
        control_time_s = 1 if args.dry_run else int(raw["dataset"]["episode_time_s"])

    runtime_robot: Any = NoSendActionRobot(robot) if args.no_send_action else robot

    logger.info(
        "Starting Quest closed-loop runtime with dry_run=%s no_send_action=%s config=%s",
        args.dry_run,
        args.no_send_action,
        args.config,
    )

    try:
        notify("Quest closed-loop runtime starting.", kind="info")
        notify("Connecting to robot bus...", kind="info")
        robot.connect()
        init_pose_deg = raw.get("init_pose_deg")
        if init_pose_deg is not None:
            if args.no_send_action:
                logger.warning(
                    "Skipping init pose ramp because --no-send-action is active."
                )
            else:
                notify("Init pose ramp starting. Keep clear.", kind="warn")
                try:
                    ramp_to_init_pose(
                        robot,
                        target_pose_deg=dict(init_pose_deg),
                        max_step_deg=float(raw.get("init_pose_max_step_deg", 0.2)),
                        timeout_s=float(raw.get("init_pose_timeout_s", 30.0)),
                        max_observed_lag_deg=raw.get(
                            "init_pose_max_observed_lag_deg", 5.0
                        ),
                    )
                except Exception:
                    notify(
                        "ABORT - init pose ramp failed. Disconnecting.",
                        kind="error",
                        urgent=True,
                    )
                    raise

                notify(
                    "Init pose reached. Wake Quest controller and prepare grip.",
                    kind="ready",
                )
                if not confirm(
                    "좌팔이 init pose에 도달했습니다. Quest 컨트롤러 깨우고 "
                    "LG 누를 준비 됐으면 OK. (Cancel = abort)"
                ):
                    notify(
                        "Operator cancelled init pose. Disconnecting.",
                        kind="error",
                        urgent=True,
                    )
                    return

        notify("Connecting Quest reader...", kind="info")
        teleop.connect()
        notify("Quest tracking armed. Press LG to start teleop.", kind="go")
        notify("Run start. Hold LG.", kind="go")
        record_loop(
            robot=runtime_robot,
            events=events,
            fps=int(raw["dataset"]["fps"]),
            teleop=teleop,
            dataset=dataset,
            control_time_s=control_time_s,
            single_task=raw["dataset"]["single_task"],
            display_data=bool(raw.get("display_data", False)),
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
        )
        notify("Run complete. Release LG.", kind="ready")
        logger.info("Record loop finished with events=%s", events)
        if dataset.has_pending_frames():
            logger.info("Saving pending Quest closed-loop episode frames.")
            dataset.save_episode()
        else:
            logger.warning("Record loop finished without pending dataset frames.")
        notify("Record loop done. Disconnecting.", kind="ready")
    except Exception:
        notify("ABORT - Quest closed-loop runtime failed.", kind="error", urgent=True)
        raise
    finally:
        _disconnect_if_connected(teleop, label="Quest teleop")
        _disconnect_if_connected(robot, label="OpenArm robot")
        dataset.finalize()


if __name__ == "__main__":
    main()
