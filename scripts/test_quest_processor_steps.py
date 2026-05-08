#!/usr/bin/env python3

from __future__ import annotations

import argparse
from importlib import import_module
import math
from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

quest_teleop_module = import_module("openarm_lerobot.quest_teleop")
QUEST_OPENARM_MOTOR_NAMES = getattr(quest_teleop_module, "QUEST_OPENARM_MOTOR_NAMES")
QUEST_OPENARM_TARGET_FRAME = getattr(quest_teleop_module, "QUEST_OPENARM_TARGET_FRAME")
QUEST_OPENARM_URDF_JOINT_NAMES = getattr(
    quest_teleop_module, "QUEST_OPENARM_URDF_JOINT_NAMES"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate Quest closed-loop processor steps."
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("assets/openarm_right.urdf"),
        help="URDF used for FK/IK validation.",
    )
    return parser.parse_args()


def build_kinematics(urdf_path: Path) -> Any:
    robot_kinematics = getattr(
        import_module("lerobot.model.kinematics"), "RobotKinematics"
    )
    return robot_kinematics(
        urdf_path=str(urdf_path),
        target_frame_name=QUEST_OPENARM_TARGET_FRAME,
        joint_names=list(QUEST_OPENARM_URDF_JOINT_NAMES),
    )


def build_observation(joints_deg: list[float]) -> dict[str, float]:
    observation: dict[str, float] = {}
    for motor_name, joint_value in zip(
        QUEST_OPENARM_MOTOR_NAMES, joints_deg, strict=True
    ):
        observation[f"{motor_name}.pos"] = float(joint_value)
        observation[f"{motor_name}.vel"] = 0.0
        observation[f"{motor_name}.torque"] = 0.0
    return observation


def build_pipeline(kinematics: Any) -> Any:
    processor_module = import_module("lerobot.processor")
    converters_module = import_module("lerobot.processor.converters")
    robot_processor_module = import_module(
        "lerobot.robots.so_follower.robot_kinematic_processor"
    )
    quest_processor_module = import_module("openarm_lerobot.quest_processor")

    robot_processor_pipeline = getattr(processor_module, "RobotProcessorPipeline")
    robot_action_observation_to_transition = getattr(
        converters_module, "robot_action_observation_to_transition"
    )
    transition_to_robot_action = getattr(
        converters_module, "transition_to_robot_action"
    )
    map_quest_action_to_robot_action = getattr(
        quest_processor_module, "MapQuestActionToRobotAction"
    )
    ee_reference_and_delta = getattr(robot_processor_module, "EEReferenceAndDelta")
    ee_bounds_and_safety = getattr(robot_processor_module, "EEBoundsAndSafety")
    gripper_velocity_to_joint = getattr(
        robot_processor_module, "GripperVelocityToJoint"
    )
    inverse_kinematics = getattr(robot_processor_module, "InverseKinematicsEEToJoints")

    motor_names = list(QUEST_OPENARM_MOTOR_NAMES)
    return robot_processor_pipeline(
        steps=[
            map_quest_action_to_robot_action(),
            ee_reference_and_delta(
                kinematics=kinematics,
                end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            ee_bounds_and_safety(
                end_effector_bounds={"min": [-2.0, -2.0, -2.0], "max": [2.0, 2.0, 2.0]},
                max_ee_step_m=0.2,
            ),
            gripper_velocity_to_joint(speed_factor=10.0, clip_min=-65.0, clip_max=0.0),
            inverse_kinematics(
                kinematics=kinematics,
                motor_names=motor_names,
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )


def build_ee_pipeline(
    kinematics: Any,
    *,
    pos_action_gain: float = 5.0,
    max_lin_vel: float = 0.3,
    fps: float = 60.0,
) -> Any:
    processor_module = import_module("lerobot.processor")
    converters_module = import_module("lerobot.processor.converters")
    quest_processor_module = import_module("openarm_lerobot.quest_processor")
    closed_loop_module = import_module("scripts.record_quest_closed_loop")

    robot_processor_pipeline = getattr(processor_module, "RobotProcessorPipeline")
    robot_action_observation_to_transition = getattr(
        converters_module, "robot_action_observation_to_transition"
    )
    transition_to_robot_action = getattr(
        converters_module, "transition_to_robot_action"
    )
    map_quest_action_to_robot_action = getattr(
        quest_processor_module, "MapQuestActionToRobotAction"
    )
    droid_ee_reference_and_delta = getattr(
        closed_loop_module, "DroidCurrentHoldEEReferenceAndDelta"
    )

    return robot_processor_pipeline(
        steps=[
            map_quest_action_to_robot_action(),
            droid_ee_reference_and_delta(
                kinematics=kinematics,
                end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                motor_names=list(QUEST_OPENARM_MOTOR_NAMES),
                use_latched_reference=True,
                pos_action_gain=pos_action_gain,
                max_lin_vel=max_lin_vel,
                fps=fps,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )


def assert_near_zero_delta_result(
    action: dict[str, float], joints_deg: list[float]
) -> None:
    for motor_name, joint_value in zip(
        QUEST_OPENARM_MOTOR_NAMES[:7], joints_deg[:7], strict=True
    ):
        solved = float(action[f"{motor_name}.pos"])
        if abs(solved - joint_value) > 1.0:
            raise AssertionError(
                f"near-zero delta produced unexpected joint jump for {motor_name}: {solved} vs {joint_value}"
            )


def extract_ee_target(transition: dict[str, Any]) -> dict[str, float]:
    action = transition["action"]
    return {
        key: float(action[key])
        for key in ["ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz", "ee.gripper_vel"]
        if key in action
    }


def run_zero_delta_case(pipeline: Any, observation: dict[str, float]) -> None:
    action = {
        "quest.pos_delta.x": 0.0,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.5,
        "quest.enabled": 1.0,
    }
    robot_action = pipeline((action, observation))
    assert_near_zero_delta_result(
        robot_action, [observation[f"{name}.pos"] for name in QUEST_OPENARM_MOTOR_NAMES]
    )


def run_bounded_delta_case(pipeline: Any, observation: dict[str, float]) -> None:
    action = {
        "quest.pos_delta.x": 0.01,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 1.0,
        "quest.enabled": 1.0,
    }
    robot_action = pipeline((action, observation))
    if not all(math.isfinite(float(value)) for value in robot_action.values()):
        raise AssertionError(
            "bounded delta case emitted non-finite robot action values"
        )
    if (
        float(robot_action["gripper.pos"]) > 0.0
        or float(robot_action["gripper.pos"]) < -65.0
    ):
        raise AssertionError(
            "bounded delta case emitted gripper position outside clip range"
        )


def run_disabled_hold_case(pipeline: Any, observation: dict[str, float]) -> None:
    action = {
        "quest.pos_delta.x": 0.02,
        "quest.pos_delta.y": 0.01,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.2,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.0,
        "quest.enabled": 0.0,
    }
    robot_action = pipeline((action, observation))
    assert_near_zero_delta_result(
        robot_action, [observation[f"{name}.pos"] for name in QUEST_OPENARM_MOTOR_NAMES]
    )
    if abs(float(robot_action["gripper.pos"]) - observation["gripper.pos"]) > 1e-6:
        raise AssertionError(
            "disabled hold case should preserve the observed gripper position"
        )


def run_stateful_latched_reference_case(
    kinematics: Any, observation: dict[str, float]
) -> None:
    pipeline = build_pipeline(kinematics)
    action = {
        "quest.pos_delta.x": 0.01,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 1.0,
        "quest.enabled": 1.0,
    }
    steps_1 = list(pipeline.step_through((action, observation)))
    first_target = extract_ee_target(steps_1[2])

    updated_observation = build_observation(
        [steps_1[-1]["action"][f"{name}.pos"] for name in QUEST_OPENARM_MOTOR_NAMES]
    )
    steps_2 = list(pipeline.step_through((action, updated_observation)))
    second_target = extract_ee_target(steps_2[2])

    for key in ["ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz"]:
        if abs(first_target[key] - second_target[key]) > 1e-6:
            raise AssertionError(
                f"latched reference drifted for {key}: {first_target[key]} vs {second_target[key]}"
            )


def run_disabled_after_tracking_case(
    kinematics: Any, observation: dict[str, float]
) -> None:
    pipeline = build_pipeline(kinematics)
    enabled_action = {
        "quest.pos_delta.x": 0.01,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 1.0,
        "quest.enabled": 1.0,
    }
    enabled_steps = list(pipeline.step_through((enabled_action, observation)))
    last_enabled_target = extract_ee_target(enabled_steps[2])
    tracked_observation = build_observation(
        [
            enabled_steps[-1]["action"][f"{name}.pos"]
            for name in QUEST_OPENARM_MOTOR_NAMES
        ]
    )

    disabled_action = {
        "quest.pos_delta.x": 0.02,
        "quest.pos_delta.y": 0.01,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.3,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.0,
        "quest.enabled": 0.0,
    }
    disabled_steps = list(pipeline.step_through((disabled_action, tracked_observation)))
    held_target = extract_ee_target(disabled_steps[2])

    for key in ["ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz"]:
        if abs(last_enabled_target[key] - held_target[key]) > 1e-6:
            raise AssertionError(
                f"disabled hold failed for {key}: {last_enabled_target[key]} vs {held_target[key]}"
            )
    if abs(held_target["ee.gripper_vel"]) > 1e-9:
        raise AssertionError("disabled after tracking should zero gripper velocity")


def run_zero_delta_holds_last_command_case() -> None:
    closed_loop_module = import_module("scripts.record_quest_closed_loop")
    hold_wrapper = getattr(closed_loop_module, "HoldWhenQuestDisabledRobotActionProcessor")
    enabled_key = getattr(closed_loop_module, "_QUEST_ENABLED_HOLD_KEY")
    zero_delta_key = getattr(closed_loop_module, "_QUEST_ZERO_DELTA_HOLD_KEY")

    class StubProcessor:
        def __call__(self, data: Any) -> dict[str, float]:
            action, _observation = data
            return {
                "joint_1.pos": float(action["joint_1_target"]),
                "joint_2.pos": 20.0,
                "joint_3.pos": 30.0,
                "joint_4.pos": 40.0,
                "joint_5.pos": 50.0,
                "joint_6.pos": 60.0,
                "joint_7.pos": 70.0,
                "gripper.pos": 0.0,
            }

    processor = hold_wrapper(StubProcessor())
    observation = build_observation([0.0] * len(QUEST_OPENARM_MOTOR_NAMES))
    first = processor(
        (
            {
                "joint_1_target": 10.0,
                enabled_key: 1.0,
                zero_delta_key: 0.0,
            },
            observation,
        )
    )

    drifted_observation = build_observation([90.0] * len(QUEST_OPENARM_MOTOR_NAMES))
    held = processor(
        (
            {
                "joint_1_target": -10.0,
                enabled_key: 1.0,
                zero_delta_key: 1.0,
            },
            drifted_observation,
        )
    )

    for key, value in first.items():
        if abs(float(held[key]) - float(value)) > 1e-9:
            raise AssertionError(
                f"zero-delta hold used observation instead of last command for {key}: "
                f"{held[key]} vs {value}"
            )


def run_fail_closed_case(pipeline: Any) -> None:
    action = {
        "quest.pos_delta.x": 0.01,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.5,
        "quest.enabled": 1.0,
    }
    try:
        pipeline((action, {}))
    except Exception:
        return
    raise AssertionError("bad observation case did not fail closed")


def run_openarm_gripper_hold_ramp_case() -> None:
    processor_module = import_module("lerobot.processor")
    quest_processor_module = import_module("openarm_lerobot.quest_processor")

    transition_key = getattr(processor_module, "TransitionKey")
    gripper_processor = getattr(
        quest_processor_module, "OpenArmGripperVelocityToJoint"
    )
    processor = gripper_processor(
        clip_min=0.0,
        clip_max=10.0,
        max_step_deg=0.5,
    )
    observation = {"gripper.pos": 0.0}
    targets: list[float] = []

    for gripper_vel in [1.0, 1.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0]:
        transition = {
            transition_key.ACTION: {"ee.gripper_vel": gripper_vel},
            transition_key.OBSERVATION: observation,
        }
        target = float(processor(transition)[transition_key.ACTION]["ee.gripper_pos"])
        targets.append(target)
        observation = {"gripper.pos": target}

    expected = [0.5, 1.0, 1.5, 1.5, 1.5, 1.0, 0.5, 0.5]
    if targets != expected:
        raise AssertionError(
            f"hold-ramp gripper targets were not rate-limited/held: {targets}"
        )


def run_droid_ee_velocity_case(kinematics: Any) -> None:
    fps = 60.0
    max_lin_vel = 0.3
    pipeline = build_ee_pipeline(
        kinematics,
        pos_action_gain=5.0,
        max_lin_vel=max_lin_vel,
        fps=fps,
    )
    observation = build_observation([0.0] * len(QUEST_OPENARM_MOTOR_NAMES))
    q_raw = [observation[f"{name}.pos"] for name in QUEST_OPENARM_MOTOR_NAMES]
    current_pose = kinematics.forward_kinematics(q_raw)
    max_step = max_lin_vel / fps

    large_action = {
        "quest.pos_delta.x": 1.0,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.5,
        "quest.enabled": 1.0,
    }
    large_target = pipeline((large_action, observation))
    large_step = math.sqrt(
        sum(
            (
                float(large_target[f"ee.{axis}"])
                - float(current_pose[index, 3])
            )
            ** 2
            for index, axis in enumerate(["x", "y", "z"])
        )
    )
    if large_step > max_step + 1e-9:
        raise AssertionError(
            f"DROID EE step exceeded velocity limit: {large_step} > {max_step}"
        )

    pipeline = build_ee_pipeline(
        kinematics,
        pos_action_gain=5.0,
        max_lin_vel=max_lin_vel,
        fps=fps,
    )
    small_action = {
        "quest.pos_delta.x": 0.001,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.5,
        "quest.enabled": 1.0,
    }
    small_target = pipeline((small_action, observation))
    small_dx = float(small_target["ee.x"]) - float(current_pose[0, 3])
    if not (0.0 < small_dx < max_step):
        raise AssertionError(f"DROID small delta did not move smoothly: dx={small_dx}")

    pipeline = build_ee_pipeline(kinematics, fps=fps)
    zero_action = {
        "quest.pos_delta.x": 0.0,
        "quest.pos_delta.y": 0.0,
        "quest.pos_delta.z": 0.0,
        "quest.rot_delta.rx": 0.0,
        "quest.rot_delta.ry": 0.0,
        "quest.rot_delta.rz": 0.0,
        "quest.gripper": 0.5,
        "quest.enabled": 1.0,
    }
    zero_target = pipeline((zero_action, observation))
    zero_step = math.sqrt(
        sum(
            (
                float(zero_target[f"ee.{axis}"])
                - float(current_pose[index, 3])
            )
            ** 2
            for index, axis in enumerate(["x", "y", "z"])
        )
    )
    if zero_step > 1e-9:
        raise AssertionError(f"DROID zero delta moved EE target: {zero_step}")


def run_quest_diagnostic_reason_case() -> None:
    read_controller_state = getattr(quest_teleop_module, "read_controller_state")
    compute_calibrated_delta = getattr(quest_teleop_module, "compute_calibrated_delta")

    class BadReader:
        def get_transforms_and_buttons(self) -> object:
            return None

    state, reason = read_controller_state(BadReader(), "left", return_reason=True)
    if state is not None or reason != "payload_not_tuple":
        raise AssertionError(f"unexpected bad payload reason: {state}, {reason}")

    identity = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    delta, reason = compute_calibrated_delta(
        identity, identity, identity, return_reason=True
    )
    if delta is None or reason != "ok":
        raise AssertionError(f"identity delta diagnostic failed: {delta}, {reason}")

    delta, reason = compute_calibrated_delta(
        object(), identity, identity, return_reason=True
    )
    if delta is not None or reason != "bad_raw_transform":
        raise AssertionError(f"unexpected raw transform reason: {delta}, {reason}")


def main() -> None:
    args = parse_args()
    kinematics = build_kinematics(args.urdf)
    pipeline = build_pipeline(kinematics)
    seed_joints = [0.0] * len(QUEST_OPENARM_MOTOR_NAMES)
    observation = build_observation(seed_joints)

    run_zero_delta_case(pipeline, observation)
    run_bounded_delta_case(pipeline, observation)
    run_disabled_hold_case(build_pipeline(kinematics), observation)
    run_stateful_latched_reference_case(kinematics, observation)
    run_disabled_after_tracking_case(kinematics, observation)
    run_zero_delta_holds_last_command_case()
    run_openarm_gripper_hold_ramp_case()
    run_droid_ee_velocity_case(kinematics)
    run_quest_diagnostic_reason_case()
    run_fail_closed_case(pipeline)
    print("Quest processor step validation passed.")


if __name__ == "__main__":
    main()
