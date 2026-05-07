#!/usr/bin/env python

"""Quest teleop that emits spatial actions only for closed-loop processing."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import monotonic, sleep
from typing import Any, cast

import numpy as np  # pyright: ignore[reportMissingImports]

from .quest_teleop import (
    QUEST_OPENARM_CALIBRATE_POLL_S,
    QUEST_OPENARM_CALIBRATE_TIMEOUT_S,
    QUEST_OPENARM_COORD_TRANSFORM_VEC,
    QUEST_OPENARM_GRIPPER_RANGE_DEG,
    QUEST_OPENARM_MAX_EE_STEP_M,
    QUEST_OPENARM_SPATIAL_SCALE,
    QUEST_OPENARM_TARGET_FRAME,
    QUEST_OPENARM_URDF_JOINT_NAMES,
    Teleoperator,
    TeleoperatorConfig,
    _QuestOpenArmTeleopConfigLike,
    _QuestReaderLike,
    _as_float_tuple,
    _clip_translation_step,
    _coord_vec_to_matrix,
    _is_pressed,
    _log_quest_debug,
    _map_trigger_to_gripper_deg,
    _normalize_controller_side,
    _resolve_quest_reader_class,
    compute_calibrated_delta,
    read_controller_state,
)


QUEST_SPATIAL_ACTION_FEATURES = (
    "quest.pos_delta.x",
    "quest.pos_delta.y",
    "quest.pos_delta.z",
    "quest.rot_delta.rx",
    "quest.rot_delta.ry",
    "quest.rot_delta.rz",
    "quest.gripper",
    "quest.enabled",
)
QUEST_SPATIAL_GRIPPER_NEUTRAL = 0.5


def _validate_spatial_contract(config: QuestSpatialTeleopConfig) -> None:
    cfg = cast(Any, config)

    if cfg.ip_address is not None:
        raise ValueError(
            "QuestSpatialTeleopConfig is USB-only; ip_address must be None."
        )

    _normalize_controller_side(cfg.controller_side)

    if cfg.target_frame != QUEST_OPENARM_TARGET_FRAME:
        raise ValueError(f"target_frame must be {QUEST_OPENARM_TARGET_FRAME!r}.")

    if cfg.urdf_joint_names != QUEST_OPENARM_URDF_JOINT_NAMES:
        raise ValueError(
            "urdf_joint_names must match the frozen seven-joint OpenArm chain."
        )

    if cfg.gripper_range_deg != QUEST_OPENARM_GRIPPER_RANGE_DEG:
        raise ValueError(
            "gripper_range_deg must match the frozen right-arm gripper range."
        )


@TeleoperatorConfig.register_subclass("quest_spatial_teleop")
@dataclass(kw_only=True)
class QuestSpatialTeleopConfig(TeleoperatorConfig):
    ip_address: str | None = None
    controller_side: str = "right"
    target_frame: str = QUEST_OPENARM_TARGET_FRAME
    urdf_joint_names: tuple[str, ...] = field(
        default_factory=lambda: QUEST_OPENARM_URDF_JOINT_NAMES
    )
    coord_transform_vec: tuple[float, ...] = field(
        default_factory=lambda: QUEST_OPENARM_COORD_TRANSFORM_VEC
    )
    spatial_scale: float = QUEST_OPENARM_SPATIAL_SCALE
    max_ee_step_m: float = QUEST_OPENARM_MAX_EE_STEP_M
    zero_orientation_delta: bool = False
    gripper_range_deg: tuple[float, ...] = field(
        default_factory=lambda: QUEST_OPENARM_GRIPPER_RANGE_DEG
    )

    def __post_init__(self) -> None:
        self.urdf_joint_names = tuple(self.urdf_joint_names)
        self.coord_transform_vec = _as_float_tuple(
            "coord_transform_vec", self.coord_transform_vec, 4
        )
        self.gripper_range_deg = _as_float_tuple(
            "gripper_range_deg", self.gripper_range_deg, 2
        )

        if self.spatial_scale <= 0.0:
            raise ValueError("spatial_scale must be a positive number.")

        if self.max_ee_step_m <= 0.0:
            raise ValueError("max_ee_step_m must be a positive number.")

        _validate_spatial_contract(self)


def _controller_command_keys(controller_side: str) -> tuple[str, str]:
    side = _normalize_controller_side(controller_side)
    if side == "l":
        return "LG", "leftTrig"
    return "RG", "rightTrig"


class QuestSpatialTeleop(Teleoperator):
    config_class: type[QuestSpatialTeleopConfig] = QuestSpatialTeleopConfig
    name: str = "quest_spatial_teleop"

    def __init__(self, config: QuestSpatialTeleopConfig):
        self._quest_config: QuestSpatialTeleopConfig = config
        cfg = cast(Any, config)
        self._reader: _QuestReaderLike | None = None
        self._connected = False
        self._state = "disconnected"
        self._ref_controller_tf: np.ndarray | None = None
        self._grip_was_pressed = False
        self._coord_transform_matrix: np.ndarray = _coord_vec_to_matrix(
            cfg.coord_transform_vec
        )
        super().__init__(config)

    @property
    def action_features(self) -> dict[str, type[float]]:
        return {key: float for key in QUEST_SPATIAL_ACTION_FEATURES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return self._state in {"calibrated_idle", "tracking"}

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return

        _validate_spatial_contract(self._quest_config)
        reader_ctor = cast(Any, _resolve_quest_reader_class())
        quest_cfg = cast(Any, self._quest_config)
        reader = cast(_QuestReaderLike, reader_ctor(ip_address=quest_cfg.ip_address))
        reader.connect()

        self._reader = reader
        self._ref_controller_tf = None
        self._grip_was_pressed = False
        self._connected = True
        self._state = "connected_uncalibrated"
        _log_quest_debug(
            event="spatial_connect", state=self._state, calibrate=calibrate
        )

        if calibrate:
            self.calibrate()

    def calibrate(self) -> None:
        if not self._connected or self._reader is None:
            raise RuntimeError("QuestSpatialTeleop is not connected.")

        deadline = monotonic() + QUEST_OPENARM_CALIBRATE_TIMEOUT_S
        while True:
            quest_cfg = cast(Any, self._quest_config)
            controller_state = read_controller_state(
                self._reader, quest_cfg.controller_side
            )
            if controller_state is None:
                if monotonic() >= deadline:
                    raise RuntimeError(
                        "Quest calibration timed out waiting for controller transforms; "
                        "check Quest / ADB / controller-transform unavailability."
                    )
                sleep(QUEST_OPENARM_CALIBRATE_POLL_S)
                continue

            controller_tf, _buttons = controller_state
            self._ref_controller_tf = controller_tf.copy()
            self._grip_was_pressed = False
            self._state = "calibrated_idle"
            _log_quest_debug(
                event="spatial_calibrate",
                state=self._state,
                ref_controller_tf=self._ref_controller_tf,
            )
            return

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, float]:
        if not self._connected or self._reader is None:
            raise RuntimeError("QuestSpatialTeleop is not connected.")

        if not self.is_calibrated or self._ref_controller_tf is None:
            raise RuntimeError(
                "QuestSpatialTeleop must be calibrated before get_action() is available."
            )

        quest_cfg = cast(Any, self._quest_config)
        controller_state = read_controller_state(
            self._reader, quest_cfg.controller_side
        )
        if controller_state is None:
            self._state = "calibrated_idle"
            self._grip_was_pressed = False
            action = self._zero_action(
                enabled=False, gripper=QUEST_SPATIAL_GRIPPER_NEUTRAL
            )
            _log_quest_debug(
                event="spatial_controller_unavailable",
                t=monotonic(),
                state=self._state,
                teleop_output=action,
            )
            return action

        controller_tf, buttons = controller_state
        grip_key, trigger_key = _controller_command_keys(quest_cfg.controller_side)
        grip_pressed = _is_pressed(buttons.get(grip_key))
        trigger_gripper = _map_trigger_to_gripper_deg(
            buttons.get(trigger_key), quest_cfg.gripper_range_deg
        )
        normalized_gripper = self._normalize_gripper(trigger_gripper)
        base_debug_payload = {
            "t": monotonic(),
            "state": self._state,
            "grip": grip_pressed,
            "controller_raw_pose": controller_tf,
            "buttons": buttons,
        }

        if not grip_pressed:
            self._state = "calibrated_idle"
            self._grip_was_pressed = False
            self._ref_controller_tf = controller_tf.copy()
            action = self._zero_action(
                enabled=False, gripper=QUEST_SPATIAL_GRIPPER_NEUTRAL
            )
            _log_quest_debug(
                event="spatial_idle_hold", **base_debug_payload, teleop_output=action
            )
            return action

        if not self._grip_was_pressed:
            self._ref_controller_tf = controller_tf.copy()

        calibrated_delta = compute_calibrated_delta(
            controller_tf,
            self._ref_controller_tf,
            self._coord_transform_matrix,
        )
        if calibrated_delta is None:
            self._state = "calibrated_idle"
            self._grip_was_pressed = True
            action = self._zero_action(
                enabled=False, gripper=QUEST_SPATIAL_GRIPPER_NEUTRAL
            )
            _log_quest_debug(
                event="spatial_delta_unavailable",
                **base_debug_payload,
                teleop_output=action,
            )
            return action

        position_delta, raw_orientation_delta = calibrated_delta
        orientation_delta = (
            np.zeros(3, dtype=float)
            if bool(quest_cfg.zero_orientation_delta)
            else raw_orientation_delta
        )
        clipped_position = _clip_translation_step(
            position_delta * float(quest_cfg.spatial_scale),
            float(quest_cfg.max_ee_step_m),
        )
        scaled_position = position_delta * float(quest_cfg.spatial_scale)
        clipped_by_max_ee_step = not np.allclose(scaled_position, clipped_position)
        self._state = "tracking"
        self._grip_was_pressed = True
        action = {
            "quest.pos_delta.x": float(clipped_position[0]),
            "quest.pos_delta.y": float(clipped_position[1]),
            "quest.pos_delta.z": float(clipped_position[2]),
            "quest.rot_delta.rx": float(orientation_delta[0]),
            "quest.rot_delta.ry": float(orientation_delta[1]),
            "quest.rot_delta.rz": float(orientation_delta[2]),
            "quest.gripper": float(normalized_gripper),
            "quest.enabled": 1.0,
        }
        _log_quest_debug(
            event="spatial_tracking",
            **base_debug_payload,
            calibrated_pos_delta=position_delta,
            calibrated_rot_delta=raw_orientation_delta,
            emitted_rot_delta=orientation_delta,
            zero_orientation_delta=bool(quest_cfg.zero_orientation_delta),
            calibrated_pos_delta_norm=float(np.linalg.norm(position_delta)),
            calibrated_rot_delta_norm=float(np.linalg.norm(raw_orientation_delta)),
            scaled_pos_delta=scaled_position,
            clipped_pos_delta=clipped_position,
            clipped_by_max_ee_step=clipped_by_max_ee_step,
            max_ee_step_m=float(quest_cfg.max_ee_step_m),
            teleop_output=action,
        )
        return action

    def send_feedback(self, feedback: dict[str, object]) -> None:
        raise NotImplementedError("Quest spatial teleop feedback is not implemented.")

    def disconnect(self) -> None:
        reader = self._reader
        self._reader = None
        self._connected = False
        self._state = "disconnected"
        self._ref_controller_tf = None
        self._grip_was_pressed = False
        _log_quest_debug(event="spatial_disconnect", state=self._state)

        if reader is not None:
            reader.disconnect()

    def _normalize_gripper(self, trigger_gripper: float | None) -> float:
        if trigger_gripper is None:
            return 0.0
        quest_cfg = cast(Any, self._quest_config)
        grip_min, grip_max = quest_cfg.gripper_range_deg
        denom = grip_max - grip_min
        if abs(denom) <= 1e-12:
            return 0.0
        normalized = (float(trigger_gripper) - grip_min) / denom
        return float(min(1.0, max(0.0, normalized)))

    def _zero_action(self, *, enabled: bool, gripper: float) -> dict[str, float]:
        return {
            "quest.pos_delta.x": 0.0,
            "quest.pos_delta.y": 0.0,
            "quest.pos_delta.z": 0.0,
            "quest.rot_delta.rx": 0.0,
            "quest.rot_delta.ry": 0.0,
            "quest.rot_delta.rz": 0.0,
            "quest.gripper": float(gripper),
            "quest.enabled": 1.0 if enabled else 0.0,
        }
