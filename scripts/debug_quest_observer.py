#!/usr/bin/env python3

from __future__ import annotations

import argparse
from importlib import import_module
import json
import logging
import math
import time
from pathlib import Path
from typing import Any


helpers = import_module("record_quest_closed_loop")

load_raw_config = getattr(helpers, "load_raw_config")
make_robot_config = getattr(helpers, "make_robot_config")
make_teleop_config = getattr(helpers, "make_teleop_config")
make_kinematics = getattr(helpers, "make_kinematics")
make_processors = getattr(helpers, "make_processors")
NoSendActionRobot = getattr(helpers, "NoSendActionRobot")
SafeOpenArmFollower = getattr(helpers, "SafeOpenArmFollower")
QuestSpatialTeleop = getattr(helpers, "QuestSpatialTeleop")


logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Observe Quest closed-loop processor internals."
    )
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to record config JSON."
    )
    parser.add_argument("--no-send-action", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--hz", type=float, default=10.0)
    return parser.parse_args()


def extract_ee_pose(mapping: dict[str, Any], prefix: str = "ee.") -> dict[str, float]:
    result: dict[str, float] = {}
    for key in ["x", "y", "z", "wx", "wy", "wz", "gripper_pos", "gripper_vel"]:
        full_key = f"{prefix}{key}"
        if full_key in mapping:
            result[full_key] = float(mapping[full_key])
    return result


def extract_joint_positions(mapping: dict[str, Any]) -> dict[str, float]:
    return {key: float(value) for key, value in mapping.items() if key.endswith(".pos")}


def compute_tracking_error(
    actual_ee: dict[str, float], target_ee: dict[str, float]
) -> float | None:
    try:
        dx = target_ee["ee.x"] - actual_ee["ee.x"]
        dy = target_ee["ee.y"] - actual_ee["ee.y"]
        dz = target_ee["ee.z"] - actual_ee["ee.z"]
    except KeyError:
        return None
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )

    raw = load_raw_config(args.config)
    robot = SafeOpenArmFollower(make_robot_config(raw))
    teleop = QuestSpatialTeleop(make_teleop_config(raw))
    kinematics = make_kinematics(
        raw["teleop"].get("urdf_path", "assets/openarm_right.urdf")
    )
    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_processors(kinematics)
    )
    runtime_robot: Any = NoSendActionRobot(robot) if args.no_send_action else robot
    period_s = 0.0 if args.hz <= 0 else 1.0 / args.hz

    try:
        robot.connect()
        teleop.connect()

        while True:
            loop_started = time.monotonic()
            obs = robot.get_observation()
            obs_processed = robot_observation_processor(obs)
            act = teleop.get_action()

            teleop_steps = list(teleop_action_processor.step_through((act, obs)))
            ik_steps = list(
                robot_action_processor.step_through(
                    (teleop_action_processor((act, obs)), obs)
                )
            )

            actual_ee = extract_ee_pose(obs_processed)
            pre_clamp_ee = (
                extract_ee_pose(teleop_steps[2]["action"])
                if len(teleop_steps) > 2
                else {}
            )
            post_clamp_ee = (
                extract_ee_pose(teleop_steps[3]["action"])
                if len(teleop_steps) > 3
                else {}
            )
            target_ee = (
                extract_ee_pose(teleop_steps[-1]["action"]) if teleop_steps else {}
            )
            ik_action = (
                extract_joint_positions(ik_steps[-1]["action"]) if ik_steps else {}
            )
            clamp_delta = {
                key: post_clamp_ee[key] - pre_clamp_ee[key]
                for key in pre_clamp_ee.keys() & post_clamp_ee.keys()
            }
            tracking_err = compute_tracking_error(actual_ee, target_ee)

            payload = {
                "actual_joints": extract_joint_positions(obs),
                "fk_actual_ee": actual_ee,
                "processor_target_ee": target_ee,
                "ik_solved_joints": ik_action,
                "clamp_pre": pre_clamp_ee,
                "clamp_post": post_clamp_ee,
                "clamp_delta": clamp_delta,
                "tracking_err": tracking_err,
            }
            logger.info("QUEST_OBSERVER %s", json.dumps(payload, sort_keys=True))

            if not args.no_send_action and ik_steps:
                runtime_robot.send_action(ik_steps[-1]["action"])

            if args.once:
                return

            remaining = period_s - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        logger.info("Stopping Quest observer on keyboard interrupt.")
    finally:
        try:
            teleop.disconnect()
        finally:
            if robot.is_connected:
                robot.disconnect()


if __name__ == "__main__":
    main()
