#!/usr/bin/env python3

from __future__ import annotations

import argparse
from importlib import import_module
import math
from pathlib import Path


kinematics_module = import_module("lerobot.model.kinematics")
quest_teleop_module = import_module("openarm_lerobot.quest_teleop")

RobotKinematics = getattr(kinematics_module, "RobotKinematics")
QUEST_OPENARM_URDF_JOINT_NAMES = getattr(
    quest_teleop_module, "QUEST_OPENARM_URDF_JOINT_NAMES"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate Quest IK/FK round-trip on the frozen URDF."
    )
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--target-frame", required=True)
    return parser.parse_args()


def max_abs_deg(values_a: list[float], values_b: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(values_a, values_b, strict=True))


def main() -> None:
    args = parse_args()
    kinematics = RobotKinematics(
        urdf_path=str(args.urdf),
        target_frame_name=args.target_frame,
        joint_names=list(QUEST_OPENARM_URDF_JOINT_NAMES),
    )

    test_vectors = [
        [0.0] * 7,
        [5.0, -2.0, 3.0, 10.0, -4.0, 2.0, 1.0],
        [-10.0, 8.0, -6.0, 15.0, 5.0, -3.0, 2.0],
    ]

    worst_error = 0.0
    for joints_deg in test_vectors:
        ee_pose = kinematics.forward_kinematics(joints_deg)
        solved = kinematics.inverse_kinematics(joints_deg, ee_pose)
        solved_list = [float(value) for value in solved[:7]]
        if not all(math.isfinite(value) for value in solved_list):
            raise AssertionError("IK round-trip produced non-finite joint values")
        error = max_abs_deg(joints_deg, solved_list)
        worst_error = max(worst_error, error)

    print(
        f"Quest IK round-trip validation passed. worst_abs_error_deg={worst_error:.4f}"
    )


if __name__ == "__main__":
    main()
