# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.motion_generation import ArticulationKinematicsSolver, interface_config_loader, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import get_extension_path_from_name
import os
from typing import Optional


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for UR5e robot.  This class loads a LulaKinematicsSovler object

    Args:
        robot_articulation (Articulation): An initialized Articulation object representing this UR5e
        end_effector_frame_name (Optional[str]): The name of the UR5e end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
        attach_gripper (Optional[bool]): If True, a URDF will be loaded that includes a suction gripper.  Defaults to False. 
    """

    def __init__(
        self,
        robot_articulation: Articulation,
        end_effector_frame_name: Optional[str] = None,
        attach_gripper: Optional[bool] = False,
    ) -> None:

        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")

        if attach_gripper:
            robot_urdf_path = os.path.join(mg_extension_path, "motion_policy_configs/ur5e/ur5e_robotiq2f85.urdf")
        else:
            robot_urdf_path = os.path.join(mg_extension_path, "motion_policy_configs/ur5e/ur5e_robotiq2f85.urdf")
        if attach_gripper:
            robot_description_yaml_path = os.path.join(
                mg_extension_path, "motion_policy_configs/ur5e/rmpflow_suction/ur5e_with_gripper_description.yaml"
            )
        else:
            robot_description_yaml_path = os.path.join(
                mg_extension_path, "motion_policy_configs/ur5e/rmpflow/ur5e_with_gripper_description.yaml"
            )

        self._kinematics = LulaKinematicsSolver(
            robot_description_path=robot_description_yaml_path, urdf_path=robot_urdf_path
        )

        if end_effector_frame_name is None:
            if attach_gripper:
                end_effector_frame_name = "tool0"
            else:
                end_effector_frame_name = "tool0"

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return
