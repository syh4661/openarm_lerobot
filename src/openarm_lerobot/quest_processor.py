#!/usr/bin/env python

"""Quest-specific processor steps for closed-loop teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from typing import Any, cast


class _FeatureTypeFallback:
    ACTION = "action"


class _PipelineFeatureTypeFallback:
    ACTION = "action"
    OBSERVATION = "observation"


@dataclass
class _PolicyFeatureFallback:
    type: object
    shape: tuple[int, ...]


class _ProcessorStepRegistryFallback:
    @classmethod
    def register(cls, *_args: object, **_kwargs: object):
        def decorator(subclass: type[object]) -> type[object]:
            return subclass

        return decorator


class _RobotActionProcessorStepFallback:
    pass


def _resolve_configs_types() -> tuple[object, object, type[object]]:
    try:
        module = import_module("lerobot.configs.types")
    except ModuleNotFoundError:
        return (
            _FeatureTypeFallback,
            _PipelineFeatureTypeFallback,
            _PolicyFeatureFallback,
        )

    feature_type = getattr(module, "FeatureType", _FeatureTypeFallback)
    pipeline_feature_type = getattr(
        module, "PipelineFeatureType", _PipelineFeatureTypeFallback
    )
    policy_feature = getattr(module, "PolicyFeature", _PolicyFeatureFallback)
    return feature_type, pipeline_feature_type, cast(type[object], policy_feature)


def _resolve_processor_types() -> tuple[type[object], type[object]]:
    try:
        module = import_module("lerobot.processor")
    except ModuleNotFoundError:
        return _ProcessorStepRegistryFallback, _RobotActionProcessorStepFallback

    registry = getattr(module, "ProcessorStepRegistry", _ProcessorStepRegistryFallback)
    step_base = getattr(
        module, "RobotActionProcessorStep", _RobotActionProcessorStepFallback
    )
    return cast(type[object], registry), cast(type[object], step_base)


FeatureType, PipelineFeatureType, PolicyFeature = _resolve_configs_types()
ProcessorStepRegistry, RobotActionProcessorStep = _resolve_processor_types()
FeatureTypeAny = cast(Any, FeatureType)
PipelineFeatureTypeAny = cast(Any, PipelineFeatureType)
PolicyFeatureAny = cast(Any, PolicyFeature)
ProcessorStepRegistryAny = cast(Any, ProcessorStepRegistry)
RobotActionProcessorStepAny = cast(Any, RobotActionProcessorStep)
RobotAction = dict[str, float]


@ProcessorStepRegistryAny.register("map_quest_action_to_robot_action")
@dataclass
class MapQuestActionToRobotAction(RobotActionProcessorStepAny):
    """Map Quest spatial teleop outputs to the so_follower EE processor contract."""

    gripper_neutral: float = 0.5
    gripper_scale: float = 2.0

    def action(self, action: RobotAction) -> RobotAction:
        enabled = bool(action.pop("quest.enabled"))
        pos_x = float(action.pop("quest.pos_delta.x"))
        pos_y = float(action.pop("quest.pos_delta.y"))
        pos_z = float(action.pop("quest.pos_delta.z"))
        rot_rx = float(action.pop("quest.rot_delta.rx"))
        rot_ry = float(action.pop("quest.rot_delta.ry"))
        rot_rz = float(action.pop("quest.rot_delta.rz"))
        gripper = float(action.pop("quest.gripper"))

        action["enabled"] = enabled
        action["target_x"] = pos_x if enabled else 0.0
        action["target_y"] = pos_y if enabled else 0.0
        action["target_z"] = pos_z if enabled else 0.0
        action["target_wx"] = rot_rx if enabled else 0.0
        action["target_wy"] = rot_ry if enabled else 0.0
        action["target_wz"] = rot_rz if enabled else 0.0
        action["gripper_vel"] = (
            (gripper - self.gripper_neutral) * self.gripper_scale if enabled else 0.0
        )
        return action

    def transform_features(
        self, features: dict[str, dict[str, Any]]
    ) -> dict[str, dict[str, Any]]:
        for feat in [
            "pos_delta.x",
            "pos_delta.y",
            "pos_delta.z",
            "rot_delta.rx",
            "rot_delta.ry",
            "rot_delta.rz",
            "gripper",
            "enabled",
        ]:
            features[PipelineFeatureTypeAny.ACTION].pop(f"quest.{feat}", None)

        for feat in [
            "enabled",
            "target_x",
            "target_y",
            "target_z",
            "target_wx",
            "target_wy",
            "target_wz",
            "gripper_vel",
        ]:
            features[PipelineFeatureTypeAny.ACTION][feat] = PolicyFeatureAny(
                type=FeatureTypeAny.ACTION, shape=(1,)
            )

        return features
