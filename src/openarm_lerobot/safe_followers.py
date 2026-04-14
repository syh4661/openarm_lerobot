from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import can

from lerobot.cameras import CameraConfig
from lerobot.motors.damiao.tables import CAN_CMD_DISABLE
from lerobot.robots.bi_openarm_follower import BiOpenArmFollower
from lerobot.robots.bi_openarm_follower.config_bi_openarm_follower import (
    BiOpenArmFollowerConfig,
)
from lerobot.robots.config import RobotConfig
from lerobot.robots.openarm_follower import OpenArmFollower
from lerobot.robots.openarm_follower.config_openarm_follower import (
    OpenArmFollowerConfig,
    OpenArmFollowerConfigBase,
)
from lerobot.teleoperators.bi_openarm_leader import BiOpenArmLeader
from lerobot.teleoperators.bi_openarm_leader.config_bi_openarm_leader import (
    BiOpenArmLeaderConfig,
)
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.openarm_leader import OpenArmLeader
from lerobot.teleoperators.openarm_leader.config_openarm_leader import (
    OpenArmLeaderConfig,
    OpenArmLeaderConfigBase,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

logger = logging.getLogger(__name__)

_SAFE_SHUTDOWN_RETRIES = 3
_SAFE_SHUTDOWN_DELAY_S = 0.05
_SAFE_SHUTDOWN_RECV_TIMEOUT_S = 0.1
_SAFE_SHUTDOWN_SETTLE_S = 0.2


class _SafeOpenArmBusShutdownMixin:
    def _build_disable_message(self, motor_name: str) -> tuple[int, can.Message]:
        motor_id = self.bus._get_motor_id(motor_name)
        recv_id = self.bus._get_motor_recv_id(motor_name)
        data = [0xFF] * 7 + [CAN_CMD_DISABLE]
        msg = can.Message(
            arbitration_id=motor_id,
            data=data,
            is_extended_id=False,
            is_fd=self.bus.use_can_fd,
        )

        if self.bus.canbus is None:
            raise RuntimeError("CAN bus is not initialized.")

        return recv_id, msg

    def _motor_shutdown_order(self) -> list[str]:
        names = list(self.bus.motors.keys())
        non_gripper = [name for name in names if name != "gripper"]
        return [*non_gripper, *(["gripper"] if "gripper" in names else [])]

    def _drain_can_responses(self, timeout_s: float = _SAFE_SHUTDOWN_DELAY_S) -> None:
        if self.bus.canbus is None:
            return

        start_time = time.time()
        while time.time() - start_time < timeout_s:
            msg = self.bus.canbus.recv(timeout=0.001)
            if msg is None:
                break

    def _safe_disable_all_motors(self) -> None:
        last_failures: list[str] = []
        ordered_motors = self._motor_shutdown_order()
        expected_recv_ids = [
            self.bus._get_motor_recv_id(motor_name) for motor_name in ordered_motors
        ]

        for _ in range(_SAFE_SHUTDOWN_RETRIES):
            last_failures = []
            self._drain_can_responses()

            for cycle in range(2):
                responses: dict[int, can.Message] = {}

                for motor_name in ordered_motors:
                    try:
                        _, msg = self._build_disable_message(motor_name)
                        self.bus.canbus.send(msg)
                    except Exception as exc:
                        last_failures.append(f"{motor_name}: {exc}")

                time.sleep(_SAFE_SHUTDOWN_DELAY_S)

                try:
                    responses = self.bus._recv_all_responses(
                        expected_recv_ids, timeout=_SAFE_SHUTDOWN_RECV_TIMEOUT_S
                    )
                except Exception as exc:
                    last_failures.append(f"recv_all disable responses failed: {exc}")

                recv_id_to_motor = {
                    self.bus._get_motor_recv_id(name): name for name in ordered_motors
                }
                for recv_id, response in responses.items():
                    motor_name = recv_id_to_motor[recv_id]
                    self.bus._process_response(motor_name, response)

                missing_recv_ids = [
                    recv_id for recv_id in expected_recv_ids if recv_id not in responses
                ]
                if cycle == 1:
                    for recv_id in missing_recv_ids:
                        last_failures.append(
                            f"{recv_id_to_motor[recv_id]}: no disable ack after repeated batch shutdown"
                        )

                time.sleep(_SAFE_SHUTDOWN_SETTLE_S)

            if not last_failures:
                self._drain_can_responses(_SAFE_SHUTDOWN_SETTLE_S)
                return

            time.sleep(_SAFE_SHUTDOWN_SETTLE_S)

        raise RuntimeError("; ".join(last_failures))

    def _disconnect_cameras(self) -> list[str]:
        shutdown_errors: list[str] = []
        for cam_name, cam in self.cameras.items():
            try:
                cam.disconnect()
            except Exception as exc:
                shutdown_errors.append(f"camera {cam_name} disconnect failed: {exc}")
        return shutdown_errors

    def _disconnect_bus_with_safe_shutdown(self, disable_torque: bool) -> None:
        shutdown_errors: list[str] = []

        if disable_torque:
            try:
                self._safe_disable_all_motors()
            except Exception as exc:
                shutdown_errors.append(f"torque disable failed: {exc}")

        try:
            self.bus.disconnect(False)
        except Exception as exc:
            shutdown_errors.append(f"bus shutdown failed: {exc}")

        shutdown_errors.extend(self._disconnect_cameras())

        if shutdown_errors:
            raise RuntimeError(
                "OpenArm safe shutdown failed. " + "; ".join(shutdown_errors)
            )


def _reuse_calibration_namespace(instance: object, namespace_name: str) -> None:
    calibration_dir = getattr(instance, "calibration_dir", None)
    calibration_fpath = getattr(instance, "calibration_fpath", None)
    robot_id = getattr(instance, "id", None)

    if calibration_dir is None or calibration_fpath is None or robot_id is None:
        return

    current_dir = Path(calibration_dir)
    target_dir = current_dir.parent / namespace_name
    target_dir.mkdir(parents=True, exist_ok=True)
    target_fpath = target_dir / f"{robot_id}.json"

    setattr(instance, "calibration_dir", target_dir)
    setattr(instance, "calibration_fpath", target_fpath)

    if target_fpath.is_file():
        instance._load_calibration(target_fpath)

    bus = getattr(instance, "bus", None)
    if bus is not None:
        setattr(bus, "calibration", getattr(instance, "calibration", {}))


@RobotConfig.register_subclass("safe_openarm_follower")
@dataclass
class SafeOpenArmFollowerConfig(OpenArmFollowerConfig):
    pass


@TeleoperatorConfig.register_subclass("safe_openarm_leader")
@dataclass
class SafeOpenArmLeaderConfig(OpenArmLeaderConfig):
    pass


@RobotConfig.register_subclass("safe_bi_openarm_follower")
@dataclass(kw_only=True)
class SafeBiOpenArmFollowerConfig(BiOpenArmFollowerConfig):
    left_arm_config: OpenArmFollowerConfigBase
    right_arm_config: OpenArmFollowerConfigBase
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@TeleoperatorConfig.register_subclass("safe_bi_openarm_leader")
@dataclass
class SafeBiOpenArmLeaderConfig(BiOpenArmLeaderConfig):
    left_arm_config: OpenArmLeaderConfigBase
    right_arm_config: OpenArmLeaderConfigBase


class SafeOpenArmFollower(_SafeOpenArmBusShutdownMixin, OpenArmFollower):
    config_class = SafeOpenArmFollowerConfig
    name = "safe_openarm_follower"

    def __init__(self, config: SafeOpenArmFollowerConfig):
        super().__init__(config)
        _reuse_calibration_namespace(self, OpenArmFollower.name)

    @check_if_not_connected
    def disconnect(self):
        self._disconnect_bus_with_safe_shutdown(
            disable_torque=self.config.disable_torque_on_disconnect
        )
        logger.info(f"{self} disconnected.")


class SafeOpenArmLeader(_SafeOpenArmBusShutdownMixin, OpenArmLeader):
    config_class = SafeOpenArmLeaderConfig
    name = "safe_openarm_leader"

    def __init__(self, config: SafeOpenArmLeaderConfig):
        super().__init__(config)
        _reuse_calibration_namespace(self, OpenArmLeader.name)

    @check_if_not_connected
    def disconnect(self) -> None:
        self._disconnect_bus_with_safe_shutdown(
            disable_torque=self.config.manual_control
        )
        logger.info(f"{self} disconnected.")


class SafeBiOpenArmFollower(BiOpenArmFollower):
    config_class = SafeBiOpenArmFollowerConfig
    name = "safe_bi_openarm_follower"

    def __init__(self, config: SafeBiOpenArmFollowerConfig):
        self.config = config

        if config.cameras:
            left_cameras = config.cameras
            right_cameras = {}
        else:
            left_cameras = config.left_arm_config.cameras
            right_cameras = config.right_arm_config.cameras

        left_arm_config = SafeOpenArmFollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            disable_torque_on_disconnect=config.left_arm_config.disable_torque_on_disconnect,
            max_relative_target=config.left_arm_config.max_relative_target,
            cameras=left_cameras,
            side=config.left_arm_config.side,
            can_interface=config.left_arm_config.can_interface,
            use_can_fd=config.left_arm_config.use_can_fd,
            can_bitrate=config.left_arm_config.can_bitrate,
            can_data_bitrate=config.left_arm_config.can_data_bitrate,
            motor_config=config.left_arm_config.motor_config,
            position_kd=config.left_arm_config.position_kd,
            position_kp=config.left_arm_config.position_kp,
            joint_limits=config.left_arm_config.joint_limits,
        )

        right_arm_config = SafeOpenArmFollowerConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            disable_torque_on_disconnect=config.right_arm_config.disable_torque_on_disconnect,
            max_relative_target=config.right_arm_config.max_relative_target,
            cameras=right_cameras,
            side=config.right_arm_config.side,
            can_interface=config.right_arm_config.can_interface,
            use_can_fd=config.right_arm_config.use_can_fd,
            can_bitrate=config.right_arm_config.can_bitrate,
            can_data_bitrate=config.right_arm_config.can_data_bitrate,
            motor_config=config.right_arm_config.motor_config,
            position_kd=config.right_arm_config.position_kd,
            position_kp=config.right_arm_config.position_kp,
            joint_limits=config.right_arm_config.joint_limits,
        )

        self.left_arm = SafeOpenArmFollower(left_arm_config)
        self.right_arm = SafeOpenArmFollower(right_arm_config)
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        raise NotImplementedError(
            "Motor ID configuration is typically done via manufacturer tools for CAN motors."
        )

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        return super().get_observation()

    @check_if_not_connected
    def send_action(
        self,
        action: RobotAction,
        custom_kp: dict[str, float] | None = None,
        custom_kd: dict[str, float] | None = None,
    ) -> RobotAction:
        return super().send_action(action, custom_kp=custom_kp, custom_kd=custom_kd)

    @check_if_not_connected
    def disconnect(self):
        errors: list[str] = []

        try:
            self.left_arm.disconnect()
        except Exception as exc:
            errors.append(f"left arm: {exc}")

        try:
            self.right_arm.disconnect()
        except Exception as exc:
            errors.append(f"right arm: {exc}")

        if errors:
            raise RuntimeError("BiOpenArm safe shutdown failed. " + "; ".join(errors))


class SafeBiOpenArmLeader(BiOpenArmLeader):
    config_class = SafeBiOpenArmLeaderConfig
    name = "safe_bi_openarm_leader"

    def __init__(self, config: SafeBiOpenArmLeaderConfig):
        self.config = config

        left_arm_config = SafeOpenArmLeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            can_interface=config.left_arm_config.can_interface,
            use_can_fd=config.left_arm_config.use_can_fd,
            can_bitrate=config.left_arm_config.can_bitrate,
            can_data_bitrate=config.left_arm_config.can_data_bitrate,
            motor_config=config.left_arm_config.motor_config,
            manual_control=config.left_arm_config.manual_control,
            position_kd=config.left_arm_config.position_kd,
            position_kp=config.left_arm_config.position_kp,
        )

        right_arm_config = SafeOpenArmLeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            can_interface=config.right_arm_config.can_interface,
            use_can_fd=config.right_arm_config.use_can_fd,
            can_bitrate=config.right_arm_config.can_bitrate,
            can_data_bitrate=config.right_arm_config.can_data_bitrate,
            motor_config=config.right_arm_config.motor_config,
            manual_control=config.right_arm_config.manual_control,
            position_kd=config.right_arm_config.position_kd,
            position_kp=config.right_arm_config.position_kp,
        )

        self.left_arm = SafeOpenArmLeader(left_arm_config)
        self.right_arm = SafeOpenArmLeader(right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        left_arm_features = self.left_arm.action_features
        right_arm_features = self.right_arm.action_features

        return {
            **{f"left_{k}": v for k, v in left_arm_features.items()},
            **{f"right_{k}": v for k, v in right_arm_features.items()},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        raise NotImplementedError(
            "Motor ID configuration is typically done via manufacturer tools for CAN motors."
        )

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        action_dict = {}

        left_action = self.left_arm.get_action()
        action_dict.update({f"left_{key}": value for key, value in left_action.items()})

        right_action = self.right_arm.get_action()
        action_dict.update(
            {f"right_{key}": value for key, value in right_action.items()}
        )

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    @check_if_not_connected
    def disconnect(self) -> None:
        errors: list[str] = []

        try:
            self.left_arm.disconnect()
        except Exception as exc:
            errors.append(f"left arm: {exc}")

        try:
            self.right_arm.disconnect()
        except Exception as exc:
            errors.append(f"right arm: {exc}")

        if errors:
            raise RuntimeError("BiOpenArm safe shutdown failed. " + "; ".join(errors))
