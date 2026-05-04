"""OpenArm-LeRobot integration package."""

from importlib import import_module
from typing import Any, cast

from openarm_lerobot.safe_followers import (
    SafeBiOpenArmFollower,
    SafeBiOpenArmFollowerConfig,
    SafeBiOpenArmLeader,
    SafeBiOpenArmLeaderConfig,
    SafeOpenArmFollower,
    SafeOpenArmFollowerConfig,
    SafeOpenArmLeader,
    SafeOpenArmLeaderConfig,
)
from openarm_lerobot.quest_teleop import QuestOpenArmTeleop, QuestOpenArmTeleopConfig


def _resolve_symbol(module_name: str, symbol_name: str) -> Any:
    module = import_module(module_name)
    return getattr(module, symbol_name)


MapQuestActionToRobotAction = cast(
    Any,
    _resolve_symbol("openarm_lerobot.quest_processor", "MapQuestActionToRobotAction"),
)
QuestSpatialTeleop = cast(
    Any, _resolve_symbol("openarm_lerobot.quest_spatial_teleop", "QuestSpatialTeleop")
)
QuestSpatialTeleopConfig = cast(
    Any,
    _resolve_symbol("openarm_lerobot.quest_spatial_teleop", "QuestSpatialTeleopConfig"),
)

__all__ = [
    "SafeBiOpenArmFollower",
    "SafeBiOpenArmFollowerConfig",
    "SafeBiOpenArmLeader",
    "SafeBiOpenArmLeaderConfig",
    "SafeOpenArmFollower",
    "SafeOpenArmFollowerConfig",
    "SafeOpenArmLeader",
    "SafeOpenArmLeaderConfig",
    "MapQuestActionToRobotAction",
    "QuestSpatialTeleop",
    "QuestSpatialTeleopConfig",
    "QuestOpenArmTeleop",
    "QuestOpenArmTeleopConfig",
]
