"""OpenArm-LeRobot integration package."""

from importlib import import_module
from typing import Any, cast

from openarm_lerobot.quest_teleop import QuestOpenArmTeleop, QuestOpenArmTeleopConfig

_SAFE_FOLLOWER_NAMES = frozenset(
    {
        "SafeBiOpenArmFollower",
        "SafeBiOpenArmFollowerConfig",
        "SafeBiOpenArmLeader",
        "SafeBiOpenArmLeaderConfig",
        "SafeOpenArmFollower",
        "SafeOpenArmFollowerConfig",
        "SafeOpenArmLeader",
        "SafeOpenArmLeaderConfig",
    }
)


def _resolve_symbol(module_name: str, symbol_name: str) -> Any:
    module = import_module(module_name)
    return getattr(module, symbol_name)


def __getattr__(name: str) -> Any:
    if name in _SAFE_FOLLOWER_NAMES:
        return _resolve_symbol("openarm_lerobot.safe_followers", name)
    raise AttributeError(f"module 'openarm_lerobot' has no attribute {name!r}")


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
