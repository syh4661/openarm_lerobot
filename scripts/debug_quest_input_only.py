#!/usr/bin/env python3

from __future__ import annotations

import argparse
from importlib import import_module
import json
import logging
import time
from pathlib import Path
from typing import Any

quest_spatial_teleop_module = import_module("openarm_lerobot.quest_spatial_teleop")
quest_teleop_module = import_module("openarm_lerobot.quest_teleop")

QuestSpatialTeleop = getattr(quest_spatial_teleop_module, "QuestSpatialTeleop")
QuestSpatialTeleopConfig = getattr(
    quest_spatial_teleop_module, "QuestSpatialTeleopConfig"
)
QuestOpenArmTeleop = getattr(quest_teleop_module, "QuestOpenArmTeleop")
QuestOpenArmTeleopConfig = getattr(quest_teleop_module, "QuestOpenArmTeleopConfig")


logger = logging.getLogger(__name__)


def load_teleop_config(config_path: Path) -> tuple[Any, Any]:
    raw = json.loads(config_path.read_text())
    teleop_raw = raw.get("teleop")
    if not isinstance(teleop_raw, dict):
        raise ValueError(f"Config {config_path} is missing a teleop object.")

    teleop_config: dict[str, Any] = dict(teleop_raw)
    teleop_type = str(teleop_config.pop("type", "quest_openarm_teleop"))

    if teleop_type == "quest_spatial_teleop":
        teleop_config.pop("initial_joint_seed_deg", None)
        teleop_config.pop("motor_names", None)
        teleop_config.pop("joint_offsets_deg", None)
        teleop_config.pop("urdf_path", None)
        return QuestSpatialTeleop, QuestSpatialTeleopConfig(**teleop_config)

    return QuestOpenArmTeleop, QuestOpenArmTeleopConfig(**teleop_config)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Debug Quest input path without robot action sending."
    )
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to record config JSON."
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Connect, calibrate, sample one action frame, then exit.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=30.0,
        help="Polling frequency for repeated sampling mode.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        force=True,
    )

    teleop_cls, config = load_teleop_config(args.config)
    teleop_ctor = teleop_cls
    teleop: Any = teleop_ctor(config)
    period_s = 0.0 if args.hz <= 0 else 1.0 / args.hz

    logger.info(
        "Starting Quest input debug with config=%s once=%s hz=%s",
        args.config,
        args.once,
        args.hz,
    )

    try:
        teleop.connect(calibrate=True)
        action = teleop.get_action()
        logger.info("Sample teleop action keys=%s", sorted(action.keys()))

        if args.once:
            return

        while True:
            loop_started = time.monotonic()
            action = teleop.get_action()
            logger.info("Loop teleop action keys=%s", sorted(action.keys()))
            remaining = period_s - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        logger.info("Stopping Quest input debug on keyboard interrupt.")
    finally:
        teleop.disconnect()


if __name__ == "__main__":
    main()
