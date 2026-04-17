#!/usr/bin/env python

"""Frozen Quest teleop config for the OpenArm MVP."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from importlib import import_module
from math import isfinite
from pathlib import Path
from typing import Any, Protocol, cast
import warnings
from time import monotonic, sleep

import numpy as np  # pyright: ignore[reportMissingImports]


class _TeleoperatorConfigFallbackBase:
    @classmethod
    def register_subclass(cls, *_args: object, **_kwargs: object):
        def decorator(subclass: type[object]) -> type[object]:
            return subclass

        return decorator


def _resolve_teleoperator_config() -> type[_TeleoperatorConfigFallbackBase]:
    try:
        module = import_module("lerobot.teleoperators.config")
    except ModuleNotFoundError:
        return _TeleoperatorConfigFallbackBase

    candidate = getattr(module, "TeleoperatorConfig", None)
    if isinstance(candidate, type):
        return cast(type[_TeleoperatorConfigFallbackBase], candidate)

    return _TeleoperatorConfigFallbackBase


TeleoperatorConfig = _resolve_teleoperator_config()


class _TeleoperatorFallbackBase:
    def __init__(self, _config: object):
        pass


def _resolve_teleoperator_base() -> type[_TeleoperatorFallbackBase]:
    try:
        module = import_module("lerobot.teleoperators.teleoperator")
    except ModuleNotFoundError:
        return _TeleoperatorFallbackBase

    candidate = getattr(module, "Teleoperator", None)
    if isinstance(candidate, type):
        return cast(type[_TeleoperatorFallbackBase], candidate)

    return _TeleoperatorFallbackBase


Teleoperator = _resolve_teleoperator_base()

QuestReader: type[object] | None = None
RobotKinematics: type[object] | None = None


def _resolve_quest_reader_class() -> type[object]:
    global QuestReader

    if QuestReader is not None:
        return QuestReader

    module = import_module("openarm_lerobot.quest_reader")
    candidate = getattr(module, "QuestReader", None)
    if not isinstance(candidate, type):
        raise TypeError("openarm_lerobot.quest_reader.QuestReader is unavailable.")

    QuestReader = candidate
    return candidate


def _resolve_robot_kinematics_class() -> type[object]:
    global RobotKinematics

    if RobotKinematics is not None:
        return RobotKinematics

    module = import_module("lerobot.model.kinematics")
    candidate = getattr(module, "RobotKinematics", None)
    if not isinstance(candidate, type):
        raise TypeError("lerobot.model.kinematics.RobotKinematics is unavailable.")

    RobotKinematics = candidate
    return candidate


class _QuestOpenArmTeleopConfigLike(Protocol):
    ip_address: str | None
    controller_side: str
    urdf_path: Path
    target_frame: str
    urdf_joint_names: tuple[str, ...]
    motor_names: tuple[str, ...]
    joint_offsets_deg: tuple[float, ...]
    coord_transform_vec: tuple[float, ...]
    spatial_scale: float
    max_ee_step_m: float
    gripper_range_deg: tuple[float, ...]
    initial_joint_seed_deg: Sequence[float]


QUEST_OPENARM_TARGET_FRAME = "openarm_hand_tcp"
QUEST_OPENARM_URDF_JOINT_NAMES = (
    "openarm_joint1",
    "openarm_joint2",
    "openarm_joint3",
    "openarm_joint4",
    "openarm_joint5",
    "openarm_joint6",
    "openarm_joint7",
)
QUEST_OPENARM_MOTOR_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "gripper",
)
QUEST_OPENARM_ACTION_FEATURE_KEYS = tuple(
    f"{motor_name}.{suffix}"
    for motor_name in QUEST_OPENARM_MOTOR_NAMES
    for suffix in ("pos", "vel", "torque")
)
QUEST_OPENARM_JOINT_OFFSETS_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
QUEST_OPENARM_COORD_TRANSFORM_VEC = (-2.0, -1.0, -3.0, 4.0)
QUEST_OPENARM_SPATIAL_SCALE = 1.0
QUEST_OPENARM_MAX_EE_STEP_M = 0.05
QUEST_OPENARM_GRIPPER_RANGE_DEG = (-65.0, 0.0)
QUEST_OPENARM_MOTOR_NAME_MAP = tuple(
    zip(QUEST_OPENARM_URDF_JOINT_NAMES, QUEST_OPENARM_MOTOR_NAMES[:7], strict=True)
)
QUEST_OPENARM_CALIBRATE_TIMEOUT_S = 10.0
QUEST_OPENARM_CALIBRATE_POLL_S = 0.05


class _QuestReaderLike(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def get_transforms_and_buttons(self) -> object: ...


class _InverseKinematicsLike(Protocol):
    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray: ...

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        position_weight: float = 1.0,
        orientation_weight: float = 0.01,
    ) -> np.ndarray: ...


def _as_float_tuple(
    name: str, values: Sequence[float], expected_len: int
) -> tuple[float, ...]:
    try:
        coerced = tuple(float(value) for value in values)
    except TypeError as exc:
        raise ValueError(
            f"{name} must be a sequence of {expected_len} numeric values."
        ) from exc

    if len(coerced) != expected_len:
        raise ValueError(f"{name} must contain exactly {expected_len} values.")

    if any(not isfinite(value) for value in coerced):
        raise ValueError(f"{name} must contain only finite numeric values.")

    return coerced


def _validate_frozen_contract(config: _QuestOpenArmTeleopConfigLike) -> None:
    if config.ip_address is not None:
        raise ValueError(
            "QuestOpenArmTeleopConfig is USB-only; ip_address must be None."
        )

    if config.controller_side != "right":
        raise ValueError(
            "QuestOpenArmTeleopConfig is right-controller-only; controller_side must be 'right'."
        )

    if config.target_frame != QUEST_OPENARM_TARGET_FRAME:
        raise ValueError(f"target_frame must be {QUEST_OPENARM_TARGET_FRAME!r}.")

    if config.urdf_joint_names != QUEST_OPENARM_URDF_JOINT_NAMES:
        raise ValueError(
            "urdf_joint_names must match the frozen seven-joint OpenArm chain."
        )

    if config.motor_names != QUEST_OPENARM_MOTOR_NAMES:
        raise ValueError(
            "motor_names must match the frozen right-arm OpenArm motor ordering."
        )

    if config.gripper_range_deg != QUEST_OPENARM_GRIPPER_RANGE_DEG:
        raise ValueError(
            "gripper_range_deg must match the frozen right-arm gripper range."
        )


def _normalize_controller_side(side: str) -> str:
    side_key = side.strip().lower()
    if side_key in {"right", "r"}:
        return "r"
    if side_key in {"left", "l"}:
        return "l"
    raise ValueError("side must be one of: 'right', 'r', 'left', or 'l'.")


def _coerce_transform_4x4(value: object) -> np.ndarray | None:
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None

    if matrix.shape != (4, 4):
        return None

    if not np.all(np.isfinite(matrix)):
        return None

    return matrix.copy()


def _coerce_joint_vector(
    name: str, values: Sequence[float], expected_len: int
) -> np.ndarray:
    try:
        vector = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{name} must be a sequence of {expected_len} numeric values."
        ) from exc

    if vector.shape != (expected_len,):
        raise ValueError(f"{name} must contain exactly {expected_len} values.")

    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite numeric values.")

    return vector.copy()


def _build_full_action(
    pos_by_motor: dict[str, float], motor_names: Sequence[str]
) -> dict[str, float]:
    action = {key: 0.0 for key in QUEST_OPENARM_ACTION_FEATURE_KEYS}
    for motor_name in motor_names:
        action[f"{motor_name}.pos"] = float(pos_by_motor[motor_name])
    return action


def _map_trigger_to_gripper_deg(
    trigger_value: object,
    gripper_range_deg: Sequence[float] = QUEST_OPENARM_GRIPPER_RANGE_DEG,
) -> float | None:
    grip_min_deg, grip_max_deg = _as_float_tuple(
        "gripper_range_deg", gripper_range_deg, 2
    )

    if isinstance(trigger_value, Sequence) and not isinstance(trigger_value, str):
        if len(trigger_value) != 1:
            return None
        trigger_value = trigger_value[0]

    if not isinstance(trigger_value, (int, float, str)):
        return None

    try:
        normalized = float(trigger_value)
    except ValueError:
        return None

    if not isfinite(normalized):
        return None

    normalized = min(1.0, max(0.0, normalized))
    return float(grip_min_deg + normalized * (grip_max_deg - grip_min_deg))


def _rotation_matrix_to_rotvec(rotation_matrix: object) -> np.ndarray | None:
    matrix = _coerce_transform_4x4(
        np.block(
            [
                [np.asarray(rotation_matrix, dtype=float), np.zeros((3, 1))],
                [np.zeros((1, 3)), np.ones((1, 1))],
            ]
        )
    )
    if matrix is None:
        return None

    rotation = matrix[:3, :3].copy()
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        return None

    determinant = float(np.linalg.det(rotation))
    if not np.isclose(determinant, 1.0, atol=1e-6):
        return None

    trace_value = float(np.trace(rotation))
    theta = float(np.arccos(np.clip((trace_value - 1.0) / 2.0, -1.0, 1.0)))

    if theta <= 1e-9:
        return np.zeros(3, dtype=float)

    if np.isclose(theta, np.pi, atol=1e-6):
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0))
        if axis[0] >= axis[1] and axis[0] >= axis[2]:
            axis[1] = np.copysign(axis[1], rotation[0, 1] + rotation[1, 0])
            axis[2] = np.copysign(axis[2], rotation[0, 2] + rotation[2, 0])
        elif axis[1] >= axis[0] and axis[1] >= axis[2]:
            axis[0] = np.copysign(axis[0], rotation[0, 1] + rotation[1, 0])
            axis[2] = np.copysign(axis[2], rotation[1, 2] + rotation[2, 1])
        else:
            axis[0] = np.copysign(axis[0], rotation[0, 2] + rotation[2, 0])
            axis[1] = np.copysign(axis[1], rotation[1, 2] + rotation[2, 1])

        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 1e-9:
            return None

        rotvec = (axis / axis_norm) * theta
    else:
        sine_theta = float(np.sin(theta))
        if abs(sine_theta) <= 1e-9:
            return None

        axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=float,
        ) / (2.0 * sine_theta)
        rotvec = axis * theta

    if rotvec.shape != (3,) or not np.all(np.isfinite(rotvec)):
        return None

    return rotvec.astype(float, copy=True)


def _rotvec_to_rotation_matrix(rotvec: object) -> np.ndarray | None:
    try:
        vector = np.asarray(rotvec, dtype=float)
    except (TypeError, ValueError):
        return None

    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None

    theta = float(np.linalg.norm(vector))
    if theta <= 1e-12:
        return np.eye(3, dtype=float)

    axis = vector / theta
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=float,
    )
    identity = np.eye(3, dtype=float)
    rotation = identity + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)
    if not np.all(np.isfinite(rotation)):
        return None

    return rotation


def _coord_vec_to_matrix(vec: Sequence[float]) -> np.ndarray:
    coord_vec = _as_float_tuple("coord_transform_vec", vec, 4)
    matrix = np.zeros((4, 4), dtype=float)
    for row_index, raw_index in enumerate(coord_vec):
        axis_index = int(abs(raw_index)) - 1
        if axis_index < 0 or axis_index >= 4:
            raise ValueError("coord_transform_vec entries must reorder axes 1..4.")
        matrix[row_index, axis_index] = float(np.sign(raw_index))

    return matrix


def _is_pressed(value: object) -> bool:
    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)):
        return bool(value)

    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "pressed", "down", "yes", "on"}

    return False


def _clip_translation_step(position_delta: np.ndarray, max_step_m: float) -> np.ndarray:
    delta = np.asarray(position_delta, dtype=float)
    norm = float(np.linalg.norm(delta))
    if norm <= max_step_m or norm <= 1e-12:
        return delta.copy()
    return (delta / norm) * max_step_m


def read_controller_state(
    reader: _QuestReaderLike, side: str
) -> tuple[np.ndarray, dict[str, object]] | None:
    controller_id = _normalize_controller_side(side)
    payload = reader.get_transforms_and_buttons()
    if not isinstance(payload, tuple) or len(payload) != 2:
        return None

    poses_raw, buttons_raw = payload
    if not isinstance(poses_raw, dict) or not isinstance(buttons_raw, dict):
        return None

    raw_transform = poses_raw.get(controller_id)
    transform = _coerce_transform_4x4(raw_transform)
    if transform is None:
        return None

    return transform, dict(buttons_raw)


def compute_calibrated_delta(
    raw_tf: object, ref_tf: object, coord_matrix: object
) -> tuple[np.ndarray, np.ndarray] | None:
    raw_matrix = _coerce_transform_4x4(raw_tf)
    ref_matrix = _coerce_transform_4x4(ref_tf)
    reorder_matrix = _coerce_transform_4x4(coord_matrix)
    if raw_matrix is None or ref_matrix is None or reorder_matrix is None:
        return None

    try:
        ref_inverse = np.linalg.inv(ref_matrix)
    except np.linalg.LinAlgError:
        return None

    calibrated = reorder_matrix @ ref_inverse @ raw_matrix
    if calibrated.shape != (4, 4) or not np.all(np.isfinite(calibrated)):
        return None

    position_delta = calibrated[:3, 3].copy()
    orientation_delta = _rotation_matrix_to_rotvec(calibrated[:3, :3])

    if orientation_delta is None:
        return None

    if position_delta.shape != (3,) or orientation_delta.shape != (3,):
        return None

    if not np.all(np.isfinite(position_delta)) or not np.all(
        np.isfinite(orientation_delta)
    ):
        return None

    return position_delta, orientation_delta


def solve_ik_to_joint_targets(
    kinematics: _InverseKinematicsLike,
    target_ee_4x4: object,
    current_joints_deg: Sequence[float],
) -> tuple[float, ...] | None:
    target_matrix = _coerce_transform_4x4(target_ee_4x4)
    if target_matrix is None:
        return None

    current_seed = _coerce_joint_vector("current_joints_deg", current_joints_deg, 7)

    try:
        solved = kinematics.inverse_kinematics(
            current_seed.copy(),
            target_matrix.copy(),
        )
    except Exception:
        return None

    try:
        solved_vector = np.asarray(solved, dtype=float)
    except (TypeError, ValueError):
        return None

    if solved_vector.ndim != 1 or solved_vector.shape[0] < 7:
        return None

    solved_vector = solved_vector[:7].copy()
    if not np.all(np.isfinite(solved_vector)):
        return None

    return tuple(float(value) for value in solved_vector)


def format_leader_like_action(
    joints_deg: Sequence[float],
    gripper_deg: float,
    motor_names: Sequence[str],
    offsets: Sequence[float],
) -> dict[str, float]:
    if tuple(motor_names) != QUEST_OPENARM_MOTOR_NAMES:
        raise ValueError("motor_names must match the frozen OpenArm motor ordering.")

    joint_vector = _coerce_joint_vector("joints_deg", joints_deg, 7)
    offset_vector = _coerce_joint_vector("offsets", offsets, 7)

    try:
        gripper_value = float(gripper_deg)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper_deg must be a finite numeric value.") from exc

    if not isfinite(gripper_value):
        raise ValueError("gripper_deg must be a finite numeric value.")

    pos_by_motor = {
        motor_name: float(joint_value + offset_value)
        for (_, motor_name), joint_value, offset_value in zip(
            QUEST_OPENARM_MOTOR_NAME_MAP,
            joint_vector,
            offset_vector,
            strict=True,
        )
    }
    pos_by_motor[motor_names[7]] = gripper_value
    return _build_full_action(pos_by_motor, motor_names)


def hold_last_action(
    last_joints: Sequence[float],
    last_gripper: float,
    motor_names: Sequence[str],
) -> dict[str, float]:
    if tuple(motor_names) != QUEST_OPENARM_MOTOR_NAMES:
        raise ValueError("motor_names must match the frozen OpenArm motor ordering.")

    joint_vector = _coerce_joint_vector("last_joints", last_joints, 7)

    try:
        gripper_value = float(last_gripper)
    except (TypeError, ValueError) as exc:
        raise ValueError("last_gripper must be a finite numeric value.") from exc

    if not isfinite(gripper_value):
        raise ValueError("last_gripper must be a finite numeric value.")

    pos_by_motor = {
        motor_name: float(joint_value)
        for (_, motor_name), joint_value in zip(
            QUEST_OPENARM_MOTOR_NAME_MAP,
            joint_vector,
            strict=True,
        )
    }
    pos_by_motor[motor_names[7]] = gripper_value
    return _build_full_action(pos_by_motor, motor_names)


@TeleoperatorConfig.register_subclass("quest_openarm_teleop")
@dataclass(kw_only=True)
class QuestOpenArmTeleopConfig(TeleoperatorConfig):
    ip_address: str | None = None
    controller_side: str = "right"
    urdf_path: Path = Path("assets/openarm_right.urdf")
    target_frame: str = QUEST_OPENARM_TARGET_FRAME
    urdf_joint_names: tuple[str, ...] = field(
        default_factory=lambda: QUEST_OPENARM_URDF_JOINT_NAMES
    )
    motor_names: tuple[str, ...] = field(
        default_factory=lambda: QUEST_OPENARM_MOTOR_NAMES
    )
    joint_offsets_deg: tuple[float, ...] = field(
        default_factory=lambda: QUEST_OPENARM_JOINT_OFFSETS_DEG
    )
    coord_transform_vec: tuple[float, ...] = field(
        default_factory=lambda: QUEST_OPENARM_COORD_TRANSFORM_VEC
    )
    spatial_scale: float = QUEST_OPENARM_SPATIAL_SCALE
    max_ee_step_m: float = QUEST_OPENARM_MAX_EE_STEP_M
    gripper_range_deg: tuple[float, ...] = field(
        default_factory=lambda: QUEST_OPENARM_GRIPPER_RANGE_DEG
    )
    initial_joint_seed_deg: Sequence[float]

    def __post_init__(self) -> None:
        self.urdf_path = Path(self.urdf_path)

        self.urdf_joint_names = tuple(self.urdf_joint_names)
        self.motor_names = tuple(self.motor_names)
        self.joint_offsets_deg = _as_float_tuple(
            "joint_offsets_deg", self.joint_offsets_deg, 7
        )
        self.coord_transform_vec = _as_float_tuple(
            "coord_transform_vec", self.coord_transform_vec, 4
        )
        self.gripper_range_deg = _as_float_tuple(
            "gripper_range_deg", self.gripper_range_deg, 2
        )
        self.initial_joint_seed_deg = _as_float_tuple(
            "initial_joint_seed_deg",
            self.initial_joint_seed_deg,
            8,
        )

        if self.spatial_scale <= 0.0:
            raise ValueError("spatial_scale must be a positive number.")

        if self.max_ee_step_m <= 0.0:
            raise ValueError("max_ee_step_m must be a positive number.")

        _validate_frozen_contract(self)


class QuestOpenArmTeleop(Teleoperator):
    config_class: type[QuestOpenArmTeleopConfig] = QuestOpenArmTeleopConfig
    name: str = "quest_openarm_teleop"

    def __init__(self, config: QuestOpenArmTeleopConfig):
        config_like = cast(_QuestOpenArmTeleopConfigLike, config)
        self._config: _QuestOpenArmTeleopConfigLike = config_like
        self._reader: _QuestReaderLike | None = None
        self._kinematics: _InverseKinematicsLike | None = None
        self._connected: bool = False
        self._state: str = "disconnected"
        self._ref_controller_tf: np.ndarray | None = None
        self._ref_ee_pose: np.ndarray | None = None
        self._last_valid_joints: tuple[float, ...] | None = None
        self._last_valid_gripper: float | None = None
        self._grip_was_pressed: bool = False
        self._coord_transform_matrix: np.ndarray = _coord_vec_to_matrix(
            config_like.coord_transform_vec
        )
        super().__init__(config)

    @property
    def action_features(self) -> dict[str, type[float]]:
        return {key: float for key in QUEST_OPENARM_ACTION_FEATURE_KEYS}

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

        _validate_frozen_contract(cast(_QuestOpenArmTeleopConfigLike, self._config))
        seed = _as_float_tuple(
            "initial_joint_seed_deg", self._config.initial_joint_seed_deg, 8
        )

        reader_ctor = cast(Any, _resolve_quest_reader_class())
        kinematics_ctor = cast(Any, _resolve_robot_kinematics_class())

        reader = cast(_QuestReaderLike, reader_ctor(ip_address=self._config.ip_address))
        reader.connect()

        try:
            self._reader = reader
            self._kinematics = cast(
                _InverseKinematicsLike,
                kinematics_ctor(
                    str(self._config.urdf_path),
                    target_frame_name=self._config.target_frame,
                    joint_names=list(self._config.urdf_joint_names),
                ),
            )

            self._last_valid_joints = tuple(float(value) for value in seed[:7])
            self._last_valid_gripper = float(seed[7])
            self._ref_ee_pose = _coerce_transform_4x4(
                self._kinematics.forward_kinematics(np.asarray(seed[:7], dtype=float))
            )
            if self._ref_ee_pose is None:
                raise RuntimeError(
                    "Quest teleop startup FK failed for initial_joint_seed_deg."
                )
        except Exception:
            reader.disconnect()
            self._reader = None
            self._kinematics = None
            self._last_valid_joints = None
            self._last_valid_gripper = None
            self._ref_ee_pose = None
            raise

        self._ref_controller_tf = None
        self._grip_was_pressed = False
        self._connected = True
        self._state = "connected_uncalibrated"

        warnings.warn(
            "QuestOpenArmTeleop reports .vel/.torque as placeholder 0.0 values, not measurements.",
            stacklevel=2,
        )

        if calibrate:
            self.calibrate()

    def calibrate(self) -> None:
        if not self._connected or self._reader is None:
            raise RuntimeError("QuestOpenArmTeleop is not connected.")

        deadline = monotonic() + QUEST_OPENARM_CALIBRATE_TIMEOUT_S
        while True:
            controller_state = read_controller_state(
                self._reader, self._config.controller_side
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
            return

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, float]:
        if not self._connected or self._reader is None:
            raise RuntimeError("QuestOpenArmTeleop is not connected.")

        if not self.is_calibrated or self._ref_controller_tf is None:
            raise RuntimeError(
                "QuestOpenArmTeleop must be calibrated before get_action() is available."
            )

        last_action = self._hold_action()
        controller_state = read_controller_state(
            self._reader, self._config.controller_side
        )
        if controller_state is None:
            self._state = "calibrated_idle"
            self._grip_was_pressed = False
            return last_action

        controller_tf, buttons = controller_state
        grip_pressed = _is_pressed(buttons.get("RG"))
        trigger_gripper = _map_trigger_to_gripper_deg(
            buttons.get("rightTrig"), self._config.gripper_range_deg
        )

        if not grip_pressed:
            self._state = "calibrated_idle"
            self._grip_was_pressed = False
            return last_action

        if not self._grip_was_pressed:
            anchored_ee_pose = _coerce_transform_4x4(
                self._require_kinematics().forward_kinematics(
                    np.asarray(self._require_last_valid_joints(), dtype=float)
                )
            )
            if anchored_ee_pose is None:
                self._state = "calibrated_idle"
                self._grip_was_pressed = False
                return last_action

            self._ref_controller_tf = controller_tf.copy()
            self._ref_ee_pose = anchored_ee_pose

        calibrated_delta = compute_calibrated_delta(
            controller_tf,
            self._ref_controller_tf,
            self._coord_transform_matrix,
        )
        if calibrated_delta is None or self._ref_ee_pose is None:
            self._state = "calibrated_idle"
            self._grip_was_pressed = True
            return last_action

        position_delta, orientation_delta = calibrated_delta
        clipped_position = _clip_translation_step(
            position_delta * float(self._config.spatial_scale),
            float(self._config.max_ee_step_m),
        )
        rotation_delta = _rotvec_to_rotation_matrix(orientation_delta)
        if rotation_delta is None:
            self._state = "calibrated_idle"
            self._grip_was_pressed = True
            return last_action

        target_pose = self._ref_ee_pose.copy()
        target_pose[:3, 3] = self._ref_ee_pose[:3, 3] + clipped_position
        target_pose[:3, :3] = rotation_delta @ self._ref_ee_pose[:3, :3]

        solved_joints = solve_ik_to_joint_targets(
            self._require_kinematics(),
            target_pose,
            self._require_last_valid_joints(),
        )
        if solved_joints is None or trigger_gripper is None:
            self._state = "calibrated_idle"
            self._grip_was_pressed = True
            return last_action

        self._last_valid_joints = tuple(float(value) for value in solved_joints)
        self._last_valid_gripper = float(trigger_gripper)
        self._state = "tracking"
        self._grip_was_pressed = True
        return hold_last_action(
            self._last_valid_joints,
            self._last_valid_gripper,
            self._config.motor_names,
        )

    def send_feedback(self, feedback: dict[str, object]) -> None:
        raise NotImplementedError("Quest teleop feedback is not implemented.")

    def disconnect(self) -> None:
        reader = self._reader
        self._reader = None
        self._kinematics = None
        self._connected = False
        self._state = "disconnected"
        self._ref_controller_tf = None
        self._grip_was_pressed = False

        if reader is None:
            return

        reader.disconnect()

    def _require_last_valid_joints(self) -> tuple[float, ...]:
        if self._last_valid_joints is None:
            raise RuntimeError("Quest teleop has no startup seed joints available.")
        return self._last_valid_joints

    def _require_last_valid_gripper(self) -> float:
        if self._last_valid_gripper is None:
            raise RuntimeError("Quest teleop has no startup seed gripper available.")
        return self._last_valid_gripper

    def _require_kinematics(self) -> _InverseKinematicsLike:
        if self._kinematics is None:
            raise RuntimeError("Quest teleop kinematics are not initialized.")
        return self._kinematics

    def _hold_action(self) -> dict[str, float]:
        return hold_last_action(
            self._require_last_valid_joints(),
            self._require_last_valid_gripper(),
            self._config.motor_names,
        )
