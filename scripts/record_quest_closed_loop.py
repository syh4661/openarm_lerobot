#!/usr/bin/env python3

from __future__ import annotations

import argparse
from importlib import import_module
import json
import logging
from pathlib import Path
from typing import Any

feature_utils_module = import_module("lerobot.datasets.feature_utils")
dataset_module = import_module("lerobot.datasets.lerobot_dataset")
pipeline_features_module = import_module("lerobot.datasets.pipeline_features")
kinematics_module = import_module("lerobot.model.kinematics")
processor_module = import_module("lerobot.processor")
converters_module = import_module("lerobot.processor.converters")
robot_kinematic_processor_module = import_module(
    "lerobot.robots.so_follower.robot_kinematic_processor"
)
record_module = import_module("lerobot.scripts.lerobot_record")
quest_processor_module = import_module("openarm_lerobot.quest_processor")
quest_spatial_teleop_module = import_module("openarm_lerobot.quest_spatial_teleop")
quest_teleop_module = import_module("openarm_lerobot.quest_teleop")
safe_followers_module = import_module("openarm_lerobot.safe_followers")

combine_feature_dicts = getattr(feature_utils_module, "combine_feature_dicts")
LeRobotDataset = getattr(dataset_module, "LeRobotDataset")
aggregate_pipeline_dataset_features = getattr(
    pipeline_features_module, "aggregate_pipeline_dataset_features"
)
create_initial_features = getattr(pipeline_features_module, "create_initial_features")
RobotKinematics = getattr(kinematics_module, "RobotKinematics")
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
QuestSpatialTeleop = getattr(quest_spatial_teleop_module, "QuestSpatialTeleop")
QuestSpatialTeleopConfig = getattr(
    quest_spatial_teleop_module, "QuestSpatialTeleopConfig"
)
QUEST_OPENARM_MOTOR_NAMES = getattr(quest_teleop_module, "QUEST_OPENARM_MOTOR_NAMES")
QUEST_OPENARM_TARGET_FRAME = getattr(quest_teleop_module, "QUEST_OPENARM_TARGET_FRAME")
QUEST_OPENARM_URDF_JOINT_NAMES = getattr(
    quest_teleop_module, "QUEST_OPENARM_URDF_JOINT_NAMES"
)
SafeOpenArmFollower = getattr(safe_followers_module, "SafeOpenArmFollower")
SafeOpenArmFollowerConfig = getattr(safe_followers_module, "SafeOpenArmFollowerConfig")


logger = logging.getLogger(__name__)


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
    return QuestSpatialTeleopConfig(**teleop_raw)


def make_kinematics(urdf_path: Path) -> Any:
    return RobotKinematics(
        urdf_path=str(urdf_path),
        target_frame_name=QUEST_OPENARM_TARGET_FRAME,
        joint_names=list(QUEST_OPENARM_URDF_JOINT_NAMES),
    )


def make_processors(kinematics: Any) -> tuple[Any, Any, Any]:
    motor_names = list(QUEST_OPENARM_MOTOR_NAMES)
    teleop_action_processor = RobotProcessorPipeline(
        steps=[
            MapQuestActionToRobotAction(),
            EEReferenceAndDelta(
                kinematics=kinematics,
                end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-2.0, -2.0, -2.0], "max": [2.0, 2.0, 2.0]},
                max_ee_step_m=0.05,
            ),
            GripperVelocityToJoint(speed_factor=10.0, clip_min=-65.0, clip_max=0.0),
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
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
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
        make_processors(kinematics)
    )
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
        robot.connect()
        teleop.connect()
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
    finally:
        try:
            teleop.disconnect()
        finally:
            robot.disconnect()
            dataset.finalize()


if __name__ == "__main__":
    main()
