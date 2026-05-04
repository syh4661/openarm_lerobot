#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections.abc import Iterable
from importlib import import_module
from pathlib import Path
from typing import Callable, NotRequired, Protocol, TypedDict, cast

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

from validate_quest_spatial_replay import (  # pyright: ignore[reportImplicitRelativeImport]
    FAIL_CLOSED_EXPECTED_BEHAVIORS,
    FAIL_CLOSED_TRACKING_STATES,
    QUEST_SPATIAL_ACTION_FEATURES,
    JsonObject,
    JsonValue,
    load_trace,
    validate_trace,
)


RobotAction = dict[str, object]
RobotObservation = dict[str, object]
JointLimits = dict[str, list[float]]


class ProcessorPipeline(Protocol):
    def __call__(self, data: object) -> dict[str, object]: ...

    def step_through(self, data: object) -> Iterable[dict[str, object]]: ...


ProcessorClass = Callable[..., ProcessorPipeline]
StepClass = Callable[..., object]
KinematicsClass = Callable[..., object]
Converter = Callable[..., dict[str, object]]


class RuntimeSymbols(TypedDict):
    QUEST_OPENARM_MOTOR_NAMES: list[str]
    QUEST_OPENARM_TARGET_FRAME: str
    QUEST_OPENARM_URDF_JOINT_NAMES: list[str]
    RobotKinematics: KinematicsClass
    RobotProcessorPipeline: ProcessorClass
    robot_action_observation_to_transition: Converter
    observation_to_transition: Converter
    transition_to_robot_action: Converter
    transition_to_observation: Converter
    MapQuestActionToRobotAction: StepClass
    EEReferenceAndDelta: StepClass
    EEBoundsAndSafety: StepClass
    GripperVelocityToJoint: StepClass
    InverseKinematicsEEToJoints: StepClass
    ForwardKinematicsJointsToEE: StepClass


class GateReport(TypedDict):
    trace_file: str
    processed_samples: int
    nan_count: int
    joint_limit_violations: int
    max_per_step_joint_delta_rad: float
    disabled_drift_m: float
    ik_failures: int
    status: str
    errors: list[str]
    schema_valid: NotRequired[object]
    schema_errors: NotRequired[list[str]]


DEFAULT_CONFIG = Path("configs/record_quest_right_nocam.json")
DEFAULT_URDF = Path("assets/openarm_right.urdf")
EE_KEYS = ("ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz")
EE_POSITION_KEYS = ("ee.x", "ee.y", "ee.z")
MAX_DISABLED_DRIFT_M = 0.005
MAX_PER_STEP_JOINT_DELTA_RAD = 0.12


class ParsedArgs(argparse.Namespace):
    trace: Path = Path()
    report: Path = Path()
    config: Path = DEFAULT_CONFIG
    urdf: Path = DEFAULT_URDF


def parse_args() -> ParsedArgs:
    parser = argparse.ArgumentParser(
        description="Replay a validated Quest spatial trace through the closed-loop processors."
    )
    _ = parser.add_argument("--trace", type=Path, required=True, help="Replay trace JSON.")
    _ = parser.add_argument("--report", type=Path, required=True, help="JSON report path.")
    _ = parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Quest recording config with joint limits.",
    )
    _ = parser.add_argument(
        "--urdf", type=Path, default=DEFAULT_URDF, help="URDF used for FK/IK."
    )
    return parser.parse_args(namespace=ParsedArgs())


def write_report(path: Path, report: GateReport) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _ = path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


def base_report(trace_path: Path) -> GateReport:
    return {
        "trace_file": str(trace_path),
        "processed_samples": 0,
        "nan_count": 0,
        "joint_limit_violations": 0,
        "max_per_step_joint_delta_rad": 0.0,
        "disabled_drift_m": 0.0,
        "ik_failures": 0,
        "status": "fail",
        "errors": [],
    }


def append_error(report: GateReport, message: str) -> None:
    report["errors"].append(message)


def increment_report_int(report: GateReport, key: str, amount: int = 1) -> None:
    if key == "processed_samples":
        report["processed_samples"] += amount
    elif key == "nan_count":
        report["nan_count"] += amount
    elif key == "joint_limit_violations":
        report["joint_limit_violations"] += amount
    elif key == "ik_failures":
        report["ik_failures"] += amount
    else:
        raise KeyError(f"unsupported integer report key: {key}")


def update_report_float_max(report: GateReport, key: str, value: float) -> None:
    if key == "max_per_step_joint_delta_rad":
        report["max_per_step_joint_delta_rad"] = max(
            report["max_per_step_joint_delta_rad"], value
        )
    elif key == "disabled_drift_m":
        report["disabled_drift_m"] = max(report["disabled_drift_m"], value)
    else:
        raise KeyError(f"unsupported float report key: {key}")


def fail_report(args: ParsedArgs, report: GateReport, message: str) -> int:
    append_error(report, message)
    report["status"] = "fail"
    write_report(args.report, report)
    _ = sys.stderr.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 1


def load_json_object(path: Path) -> dict[str, object]:
    loaded = cast(object, json.loads(path.read_text()))
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return cast(dict[str, object], loaded)


def get_symbol(module: object, name: str) -> object:
    return cast(object, getattr(module, name))


def as_float(value: object) -> float:
    return float(cast(str | int | float, value))


def require_runtime_symbols() -> RuntimeSymbols:
    try:
        quest_teleop_module = import_module("openarm_lerobot.quest_teleop")
        kinematics_module = import_module("lerobot.model.kinematics")
        processor_module = import_module("lerobot.processor")
        converters_module = import_module("lerobot.processor.converters")
        robot_processor_module = import_module(
            "lerobot.robots.so_follower.robot_kinematic_processor"
        )
        quest_processor_module = import_module("openarm_lerobot.quest_processor")
    except ModuleNotFoundError as exc:
        missing = exc.name or str(exc)
        raise RuntimeError(
            "missing required runtime dependency; activate the Python 3.12 sibling "
            + f"LeRobot/OpenArm environment before running this gate: {missing}"
        ) from exc

    return RuntimeSymbols(
        QUEST_OPENARM_MOTOR_NAMES=cast(
            list[str], get_symbol(quest_teleop_module, "QUEST_OPENARM_MOTOR_NAMES")
        ),
        QUEST_OPENARM_TARGET_FRAME=cast(
            str, get_symbol(quest_teleop_module, "QUEST_OPENARM_TARGET_FRAME")
        ),
        QUEST_OPENARM_URDF_JOINT_NAMES=cast(
            list[str], get_symbol(quest_teleop_module, "QUEST_OPENARM_URDF_JOINT_NAMES")
        ),
        RobotKinematics=cast(
            KinematicsClass, get_symbol(kinematics_module, "RobotKinematics")
        ),
        RobotProcessorPipeline=cast(
            ProcessorClass, get_symbol(processor_module, "RobotProcessorPipeline")
        ),
        robot_action_observation_to_transition=cast(
            Converter,
            get_symbol(converters_module, "robot_action_observation_to_transition"),
        ),
        observation_to_transition=cast(
            Converter, get_symbol(converters_module, "observation_to_transition")
        ),
        transition_to_robot_action=cast(
            Converter, get_symbol(converters_module, "transition_to_robot_action")
        ),
        transition_to_observation=cast(
            Converter, get_symbol(converters_module, "transition_to_observation")
        ),
        MapQuestActionToRobotAction=cast(
            StepClass, get_symbol(quest_processor_module, "MapQuestActionToRobotAction")
        ),
        EEReferenceAndDelta=cast(
            StepClass, get_symbol(robot_processor_module, "EEReferenceAndDelta")
        ),
        EEBoundsAndSafety=cast(
            StepClass, get_symbol(robot_processor_module, "EEBoundsAndSafety")
        ),
        GripperVelocityToJoint=cast(
            StepClass, get_symbol(robot_processor_module, "GripperVelocityToJoint")
        ),
        InverseKinematicsEEToJoints=cast(
            StepClass, get_symbol(robot_processor_module, "InverseKinematicsEEToJoints")
        ),
        ForwardKinematicsJointsToEE=cast(
            StepClass, get_symbol(robot_processor_module, "ForwardKinematicsJointsToEE")
        ),
    )


def build_processors(
    symbols: RuntimeSymbols, urdf_path: Path
) -> tuple[ProcessorPipeline, ProcessorPipeline, ProcessorPipeline]:
    motor_names = list(symbols["QUEST_OPENARM_MOTOR_NAMES"])
    kinematics = symbols["RobotKinematics"](
        urdf_path=str(urdf_path),
        target_frame_name=symbols["QUEST_OPENARM_TARGET_FRAME"],
        joint_names=list(symbols["QUEST_OPENARM_URDF_JOINT_NAMES"]),
    )
    pipeline_cls = symbols["RobotProcessorPipeline"]
    teleop_action_processor = pipeline_cls(
        steps=[
            symbols["MapQuestActionToRobotAction"](),
            symbols["EEReferenceAndDelta"](
                kinematics=kinematics,
                end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                motor_names=motor_names,
                use_latched_reference=True,
            ),
            symbols["EEBoundsAndSafety"](
                end_effector_bounds={"min": [-2.0, -2.0, -2.0], "max": [2.0, 2.0, 2.0]},
                max_ee_step_m=0.05,
            ),
            symbols["GripperVelocityToJoint"](
                speed_factor=10.0, clip_min=-65.0, clip_max=0.0
            ),
        ],
        to_transition=symbols["robot_action_observation_to_transition"],
        to_output=symbols["transition_to_robot_action"],
    )
    robot_action_processor = pipeline_cls(
        steps=[
            symbols["InverseKinematicsEEToJoints"](
                kinematics=kinematics,
                motor_names=motor_names,
                initial_guess_current_joints=True,
            )
        ],
        to_transition=symbols["robot_action_observation_to_transition"],
        to_output=symbols["transition_to_robot_action"],
    )
    robot_observation_processor = pipeline_cls(
        steps=[
            symbols["ForwardKinematicsJointsToEE"](
                kinematics=kinematics, motor_names=motor_names
            )
        ],
        to_transition=symbols["observation_to_transition"],
        to_output=symbols["transition_to_observation"],
    )
    return teleop_action_processor, robot_action_processor, robot_observation_processor


def build_observation(motor_names: list[str], joints_deg: list[float]) -> dict[str, float]:
    observation: dict[str, float] = {}
    for motor_name, joint_value in zip(motor_names, joints_deg, strict=True):
        observation[f"{motor_name}.pos"] = float(joint_value)
        observation[f"{motor_name}.vel"] = 0.0
        observation[f"{motor_name}.torque"] = 0.0
    return observation


def action_from_sample(sample: JsonObject) -> dict[str, float]:
    return {
        field: float(cast(int | float, sample[field]))
        for field in QUEST_SPATIAL_ACTION_FEATURES
    }


def finite_count(values: dict[str, object]) -> int:
    return sum(
        1
        for value in values.values()
        if isinstance(value, (int, float))
        and not isinstance(value, bool)
        and not math.isfinite(float(value))
    )


def extract_joints(action: dict[str, object], motor_names: list[str]) -> list[float]:
    return [as_float(action[f"{motor_name}.pos"]) for motor_name in motor_names]


def max_joint_delta_rad(previous_deg: list[float], current_deg: list[float]) -> float:
    return max(
        math.radians(abs(current - previous))
        for previous, current in zip(previous_deg[:7], current_deg[:7], strict=True)
    )


def max_ee_position_delta_m(a: dict[str, float], b: dict[str, float]) -> float:
    return math.sqrt(sum((float(a[key]) - float(b[key])) ** 2 for key in EE_POSITION_KEYS))


def extract_ee(observation_or_action: dict[str, object]) -> dict[str, float]:
    return {
        key: as_float(observation_or_action[key])
        for key in EE_KEYS
        if key in observation_or_action
    }


def is_fail_closed_sample(sample: JsonObject) -> bool:
    return (
        sample.get("tracking_state") in FAIL_CLOSED_TRACKING_STATES
        or sample.get("quest.enabled") == 0
        or sample.get("expected_behavior") in FAIL_CLOSED_EXPECTED_BEHAVIORS
    )


def count_joint_limit_violations(
    action: dict[str, object], joint_limits: JointLimits, tolerance_deg: float = 1e-6
) -> int:
    violations = 0
    for joint_name, raw_limits in joint_limits.items():
        key = f"{joint_name}.pos"
        if key not in action:
            continue
        lower, upper = float(raw_limits[0]), float(raw_limits[1])
        value = as_float(action[key])
        if value < lower - tolerance_deg or value > upper + tolerance_deg:
            violations += 1
    return violations


def validate_intermediate_steps(
    steps: list[dict[str, object]], sample_index: int, errors: list[str]
) -> None:
    mapped = cast(dict[str, object], steps[1]["action"])
    bounded = cast(dict[str, object], steps[3]["action"])
    gripper = cast(dict[str, object], steps[4]["action"])
    for key in ("target_x", "target_y", "target_z", "target_wx", "target_wy", "target_wz"):
        if key not in mapped:
            errors.append(f"samples[{sample_index}] missing mapped key {key}")
    for key in EE_KEYS:
        if key not in bounded:
            errors.append(f"samples[{sample_index}] missing bounded EE key {key}")
    if "ee.gripper_pos" not in gripper:
        errors.append(f"samples[{sample_index}] missing converted ee.gripper_pos")
    elif not -65.0 <= as_float(gripper["ee.gripper_pos"]) <= 0.0:
        errors.append(f"samples[{sample_index}] ee.gripper_pos outside [-65, 0]")


def run_gate(args: ParsedArgs, report: GateReport) -> int:
    trace, load_error = load_trace(args.trace)
    if load_error is not None:
        return fail_report(args, report, load_error)
    if trace is None:
        return fail_report(args, report, "trace loader returned no trace")

    validation_report, validation_errors = validate_trace(trace, str(args.trace))
    report["schema_valid"] = validation_report["schema_valid"]
    if validation_errors:
        report["schema_errors"] = validation_errors
        return fail_report(args, report, "replay schema validation failed")

    try:
        config = load_json_object(args.config)
        symbols = require_runtime_symbols()
        teleop_processor, robot_action_processor, observation_processor = build_processors(
            symbols, args.urdf
        )
    except Exception as exc:
        return fail_report(args, report, str(exc))

    motor_names = list(symbols["QUEST_OPENARM_MOTOR_NAMES"])
    robot_config = cast(dict[str, object], config["robot"])
    joint_limits = cast(JointLimits, robot_config["joint_limits"])
    current_joints = [0.0] * len(motor_names)
    samples = cast(list[JsonValue], trace["samples"])

    for index, raw_sample in enumerate(samples):
        sample = cast(JsonObject, raw_sample)
        previous_target_joints = list(current_joints)
        current_observation = build_observation(motor_names, current_joints)
        action = action_from_sample(sample)

        try:
            teleop_steps = list(
                teleop_processor.step_through((action, dict(current_observation)))
            )
            validate_intermediate_steps(teleop_steps, index, report["errors"])
            ee_action = cast(dict[str, object], teleop_steps[-1]["action"])
            robot_action = robot_action_processor((ee_action, dict(current_observation)))
        except Exception as exc:
            increment_report_int(report, "ik_failures")
            append_error(report, f"samples[{index}] processor failure: {exc}")
            continue

        increment_report_int(report, "processed_samples")
        increment_report_int(
            report, "nan_count", finite_count(ee_action) + finite_count(robot_action)
        )
        increment_report_int(
            report,
            "joint_limit_violations",
            count_joint_limit_violations(robot_action, joint_limits),
        )

        next_joints = extract_joints(robot_action, motor_names)
        update_report_float_max(
            report,
            "max_per_step_joint_delta_rad",
            max_joint_delta_rad(current_joints, next_joints),
        )

        if is_fail_closed_sample(sample):
            held_observation = observation_processor(
                build_observation(motor_names, next_joints)
            )
            previous_target_observation = observation_processor(
                build_observation(motor_names, previous_target_joints)
            )
            update_report_float_max(
                report,
                "disabled_drift_m",
                max_ee_position_delta_m(
                    extract_ee(previous_target_observation), extract_ee(held_observation)
                ),
            )
            for motor_name, previous, current in zip(
                motor_names, previous_target_joints, next_joints, strict=True
            ):
                if abs(previous - current) > 1e-6:
                    append_error(
                        report,
                        f"samples[{index}] disabled/failure sample moved {motor_name}: {previous} -> {current}"
                    )

        current_joints = next_joints

    if report["processed_samples"] != len(samples):
        append_error(
            report,
            f"processed {report['processed_samples']} of {len(samples)} samples"
        )
    if report["nan_count"]:
        append_error(report, f"non-finite output count: {report['nan_count']}")
    if report["joint_limit_violations"]:
        append_error(
            report,
            f"joint limit violation count: {report['joint_limit_violations']}"
        )
    if report["disabled_drift_m"] > MAX_DISABLED_DRIFT_M:
        append_error(
            report,
            f"disabled drift {report['disabled_drift_m']:.6f} m exceeds {MAX_DISABLED_DRIFT_M:.6f} m"
        )
    if report["max_per_step_joint_delta_rad"] > MAX_PER_STEP_JOINT_DELTA_RAD:
        append_error(
            report,
            "max per-step joint delta "
            + f"{report['max_per_step_joint_delta_rad']:.6f} rad exceeds "
            + f"{MAX_PER_STEP_JOINT_DELTA_RAD:.6f} rad"
        )

    report["status"] = "pass" if not report["errors"] else "fail"
    write_report(args.report, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 1


def main() -> int:
    args = parse_args()
    report = base_report(args.trace)
    return run_gate(args, report)


if __name__ == "__main__":
    raise SystemExit(main())
