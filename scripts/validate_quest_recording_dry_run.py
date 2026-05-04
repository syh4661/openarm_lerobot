#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections.abc import Iterator
from contextlib import contextmanager
import json
import os
import signal
import sys
from importlib import import_module
from pathlib import Path
from types import MethodType
from typing import Any, cast


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


DEFAULT_CONFIG = Path("configs/record_quest_right_nocam.json")


class ParsedArgs(argparse.Namespace):
    config: Path = DEFAULT_CONFIG
    report: Path = Path()
    control_time_s: int = 1
    stage_timeout_s: int = 15


def parse_args() -> ParsedArgs:
    parser = argparse.ArgumentParser(
        description="Validate Quest recording dry-run wiring without sending actions."
    )
    _ = parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Quest recording config JSON.",
    )
    _ = parser.add_argument("--report", type=Path, required=True, help="JSON report path.")
    _ = parser.add_argument(
        "--control-time-s",
        type=int,
        default=1,
        help="Short dry-run control-loop duration in seconds.",
    )
    _ = parser.add_argument(
        "--stage-timeout-s",
        type=int,
        default=15,
        help="Fail-safe timeout for blocking hardware stages.",
    )
    return parser.parse_args(namespace=ParsedArgs())


def write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _ = path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


def print_report(report: dict[str, Any]) -> None:
    print(json.dumps(report, indent=2, sort_keys=True))


def base_report(config_path: Path) -> dict[str, Any]:
    return {
        "config": str(config_path),
        "config_available": False,
        "runtime_import_available": False,
        "robot_available": False,
        "quest_available": False,
        "dataset_available": False,
        "forced_dry_run": True,
        "forced_no_send_action": True,
        "send_action_calls": 0,
        "no_send_action_wrapper_calls": 0,
        "processor_pipeline_present": False,
        "record_loop_started": False,
        "dataset_push_enabled": None,
        "status": "fail",
        "reason": "not_started",
        "errors": [],
    }


def fail(
    report_path: Path,
    report: dict[str, Any],
    reason: str,
    message: str | None = None,
) -> int:
    report["status"] = "fail"
    report["reason"] = reason
    if message:
        cast(list[str], report["errors"]).append(message)
    write_report(report_path, report)
    print_report(report)
    return 1


def pass_gate(report_path: Path, report: dict[str, Any]) -> int:
    report["status"] = "pass"
    report["reason"] = "dry_run_record_loop_completed"
    write_report(report_path, report)
    print_report(report)
    return 0


def load_json_object(path: Path) -> dict[str, Any]:
    loaded = json.loads(path.read_text())
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return cast(dict[str, Any], loaded)


def dataset_push_enabled(raw: dict[str, Any]) -> bool:
    dataset = raw.get("dataset")
    config_enabled = bool(dataset.get("push_to_hub", False)) if isinstance(dataset, dict) else False
    env_override = os.environ.get("OPENARM_RECORD_PUSH_TO_HUB")
    env_enabled = env_override not in (None, "", "0", "false", "False", "FALSE")
    return config_enabled or env_enabled


def prepare_dry_run_config(raw: dict[str, Any], report_path: Path) -> dict[str, Any]:
    dry_run_raw = dict(raw)
    dataset_raw = dry_run_raw.get("dataset")
    if not isinstance(dataset_raw, dict):
        raise ValueError("config must contain a dataset object")

    dry_run_dataset = dict(dataset_raw)
    dry_run_name = f"{report_path.stem}-{os.getpid()}"
    dry_run_root = REPO_ROOT / ".tmp" / "quest_recording_dry_run" / dry_run_name
    dry_run_dataset["root"] = str(dry_run_root)
    dry_run_dataset["repo_id"] = f"local/{dry_run_name}"
    dry_run_dataset["push_to_hub"] = False
    dry_run_dataset["video"] = False
    dry_run_raw["dataset"] = dry_run_dataset
    return dry_run_raw


def classify_exception(stage: str, exc: BaseException) -> tuple[str, str]:
    message = f"{stage}: {type(exc).__name__}: {exc}"
    if isinstance(exc, ModuleNotFoundError):
        return "runtime_unavailable", message
    if stage in {"teleop_config", "teleop_construct", "teleop_connect"}:
        return "quest_unavailable", message
    if stage in {"robot_config", "robot_construct", "robot_connect"}:
        return "robot_unavailable", message
    if stage in {"kinematics", "processors"}:
        return "processor_unavailable", message
    if stage == "dataset":
        return "dataset_unavailable", message
    if stage == "record_loop":
        return "record_loop_error", message
    return "validation_error", message


@contextmanager
def timeout_after(seconds: int, stage: str) -> Iterator[None]:
    def handle_timeout(_signum: int, _frame: object) -> None:
        raise TimeoutError(f"{stage} timed out after {seconds} seconds")

    previous_handler = signal.signal(signal.SIGALRM, handle_timeout)
    _ = signal.alarm(seconds)
    try:
        yield
    finally:
        _ = signal.alarm(0)
        _ = signal.signal(signal.SIGALRM, previous_handler)


def install_send_action_guard(robot: Any, report: dict[str, Any]) -> None:
    def blocked_send_action(_self: Any, action: dict[str, float]) -> dict[str, float]:
        report["send_action_calls"] = int(report["send_action_calls"]) + 1
        keys = sorted(action.keys()) if isinstance(action, dict) else []
        raise RuntimeError(
            "unsafe real robot send_action attempted during dry-run; "
            + f"action keys={keys}"
        )

    setattr(robot, "send_action", MethodType(blocked_send_action, robot))


def wrap_no_send_action_robot(recorder_module: Any, report: dict[str, Any]) -> None:
    original_send_action = recorder_module.NoSendActionRobot.send_action

    def counted_no_send_action(self: Any, action: dict[str, float]) -> dict[str, float]:
        report["no_send_action_wrapper_calls"] = (
            int(report["no_send_action_wrapper_calls"]) + 1
        )
        return cast(dict[str, float], original_send_action(self, action))

    recorder_module.NoSendActionRobot.send_action = counted_no_send_action


def run_gate(args: ParsedArgs, report: dict[str, Any]) -> int:
    robot: Any | None = None
    teleop: Any | None = None
    dataset: Any | None = None
    stage = "config"
    failure_reason: str | None = None
    failure_message: str | None = None
    try:
        raw = load_json_object(args.config)
        report["config_available"] = True
        report["dataset_push_enabled"] = dataset_push_enabled(raw)
        if report["dataset_push_enabled"]:
            return fail(
                args.report,
                report,
                "dataset_push_enabled",
                "dataset push is enabled by config or environment; refusing dry-run gate",
            )
        raw = prepare_dry_run_config(raw, args.report)

        stage = "runtime_import"
        recorder_module = import_module("record_quest_closed_loop")
        report["runtime_import_available"] = True
        wrap_no_send_action_robot(recorder_module, report)

        stage = "robot_config"
        robot_config = recorder_module.make_robot_config(raw)
        stage = "teleop_config"
        teleop_config = recorder_module.make_teleop_config(raw)
        stage = "robot_construct"
        robot = recorder_module.SafeOpenArmFollower(robot_config)
        install_send_action_guard(robot, report)
        stage = "teleop_construct"
        teleop = recorder_module.QuestSpatialTeleop(teleop_config)
        stage = "kinematics"
        kinematics = recorder_module.make_kinematics(
            Path(raw["teleop"].get("urdf_path", "assets/openarm_right.urdf"))
        )
        stage = "processors"
        (
            teleop_action_processor,
            robot_action_processor,
            robot_observation_processor,
        ) = recorder_module.make_processors(kinematics)
        report["processor_pipeline_present"] = all(
            processor is not None
            for processor in (
                teleop_action_processor,
                robot_action_processor,
                robot_observation_processor,
            )
        )

        stage = "dataset"
        dataset = recorder_module.make_dataset(
            raw, robot, teleop, teleop_action_processor, robot_observation_processor
        )
        report["dataset_available"] = True
        events = {"exit_early": False, "stop_recording": False, "rerecord_episode": False}
        runtime_robot = recorder_module.NoSendActionRobot(robot)

        stage = "robot_connect"
        assert robot is not None
        with timeout_after(args.stage_timeout_s, stage):
            robot.connect()
        report["robot_available"] = True
        stage = "teleop_connect"
        assert teleop is not None
        with timeout_after(args.stage_timeout_s, stage):
            teleop.connect()
        report["quest_available"] = True

        stage = "record_loop"
        report["record_loop_started"] = True
        with timeout_after(args.stage_timeout_s + args.control_time_s, stage):
            recorder_module.record_loop(
                robot=runtime_robot,
                events=events,
                fps=int(raw["dataset"]["fps"]),
                teleop=teleop,
                dataset=dataset,
                control_time_s=args.control_time_s,
                single_task=raw["dataset"]["single_task"],
                display_data=bool(raw.get("display_data", False)),
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )
    except Exception as exc:
        failure_reason, failure_message = classify_exception(stage, exc)
        if int(report["send_action_calls"]):
            failure_reason = "send_action_called"
    finally:
        if teleop is not None and bool(report["quest_available"]):
            try:
                teleop.disconnect()
            except Exception as exc:
                cast(list[str], report["errors"]).append(
                    f"teleop_disconnect: {type(exc).__name__}: {exc}"
                )
        if robot is not None and bool(report["robot_available"]):
            try:
                robot.disconnect()
            except Exception as exc:
                cast(list[str], report["errors"]).append(
                    f"robot_disconnect: {type(exc).__name__}: {exc}"
                )
        if dataset is not None and bool(report["dataset_available"]):
            try:
                dataset.finalize()
            except Exception as exc:
                cast(list[str], report["errors"]).append(
                    f"dataset_finalize: {type(exc).__name__}: {exc}"
                )

    if failure_reason is not None:
        return fail(args.report, report, failure_reason, failure_message)

    if int(report["send_action_calls"]):
        return fail(
            args.report,
            report,
            "send_action_called",
            "real robot send_action was called during dry-run",
        )
    return pass_gate(args.report, report)


def main() -> int:
    args = parse_args()
    report = base_report(args.config)
    try:
        return run_gate(args, report)
    except Exception as exc:
        return fail(
            args.report,
            report,
            "validation_error",
            f"unhandled validator error: {type(exc).__name__}: {exc}",
        )


if __name__ == "__main__":
    raise SystemExit(main())
