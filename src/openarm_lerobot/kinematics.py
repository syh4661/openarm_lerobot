"""OpenArm-specific kinematics helpers."""

from __future__ import annotations

import numpy as np
from lerobot.model.kinematics import RobotKinematics


class OpenArmKinematics(RobotKinematics):
    """RobotKinematics with a weak posture task for redundant OpenArm joints."""

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str,
        joint_names: list[str],
        posture_weight: float = 0.005,
        posture_target_deg: dict[str, float] | None = None,
        anchor_to_current: bool = True,
    ) -> None:
        super().__init__(
            urdf_path=urdf_path,
            target_frame_name=target_frame_name,
            joint_names=joint_names,
        )
        self._anchor_to_current = anchor_to_current
        self.posture_task = self.solver.add_joints_task()
        self.posture_task.configure("posture", "soft", posture_weight)

        targets = {joint_name: 0.0 for joint_name in self.joint_names}
        if posture_target_deg is not None:
            targets.update(posture_target_deg)
        self.posture_task.set_joints(
            {
                joint_name: np.deg2rad(targets[joint_name])
                for joint_name in self.joint_names
            }
        )

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        position_weight: float = 1.0,
        orientation_weight: float = 0.01,
    ) -> np.ndarray:
        if self._anchor_to_current:
            self.posture_task.set_joints(
                {
                    joint_name: float(np.deg2rad(current_joint_pos[index]))
                    for index, joint_name in enumerate(self.joint_names)
                }
            )
        return super().inverse_kinematics(
            current_joint_pos,
            desired_ee_pose,
            position_weight,
            orientation_weight,
        )
