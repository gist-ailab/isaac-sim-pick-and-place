# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, List
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
import carb
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper


class UR5e(Robot):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "ur5e_robot".
            usd_path (Optional[str], optional): [description]. Defaults to None.
            position (Optional[np.ndarray], optional): [description]. Defaults to None.
            orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            end_effector_prim_name (Optional[str], optional): [description]. Defaults to None.
            attach_gripper (bool, optional): [description]. Defaults to False.
            gripper_usd (Optional[str], optional): [description]. Defaults to "default".
            gripper_dof_names (Optional[List[str]], optional): [description]. Defaults to None.
            gripper_open_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            gripper_closed_position (Optional[np.ndarray], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        prim_path: str,
        name: str = "ur5e_robot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
        attach_gripper: bool = False,
        gripper_usd: Optional[str] = "default",
        gripper_dof_names: Optional[List[str]] = None,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
        deltas: Optional[np.ndarray] = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name
        if not prim.IsValid():  ## This is the default path
            if usd_path:
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:   ## This is the default path
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
                # usd_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            if self._end_effector_prim_name is None:    ## This is the default path
                self._end_effector_prim_path = prim_path + "/tool0"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
            if gripper_dof_names is None:   ## This is the default path
                gripper_dof_names = ["left_inner_knuckle_joint", "right_inner_knuckle_joint"]
            if gripper_open_position is None:
                gripper_open_position = np.array([0.05, 0.05]) / get_stage_units()
            if gripper_closed_position is None:
                gripper_closed_position = np.array([0.0, 0.0])
        else:
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/tool0"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
            if gripper_dof_names is None:
                gripper_dof_names = ["left_inner_knuckle_joint", "right_inner_knuckle_joint"]
            if gripper_open_position is None:
                gripper_open_position = np.array([0.05, 0.05]) / get_stage_units()
            if gripper_closed_position is None:
                gripper_closed_position = np.array([0.0, 0.0])
        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None
        )
        self._gripper_usd = gripper_usd
        if attach_gripper:
            if gripper_usd == "default":
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                    return
                # gripper_usd = assets_root_path + "/Isaac/Robots/Robotiq/2F-85/2f85_instanceable.usd"
                gripper_usd = '/home/hse/Desktop/ur5e_gripper/2F-85/2f85_instanceable.usd'
                add_reference_to_stage(usd_path=gripper_usd, prim_path=self._end_effector_prim_path)
                if deltas is None:
                    deltas = np.array([0.05, 0.05]) / get_stage_units()
                self._gripper = ParallelGripper(
                    end_effector_prim_path=self._end_effector_prim_path,
                    joint_prim_names=gripper_dof_names,
                    joint_opened_positions=gripper_open_position,
                    joint_closed_positions=gripper_closed_position,
                    action_deltas=deltas,
                )
            elif gripper_usd is None:
                carb.log_warn("Not adding a gripper usd, the gripper already exists in the ur5e asset")
                if deltas is None:
                    deltas = np.array([0.05, 0.05]) / get_stage_units()
                self._gripper = ParallelGripper(
                    end_effector_prim_path=self._end_effector_prim_path,
                    joint_prim_names=gripper_dof_names,
                    joint_opened_positions=gripper_open_position,
                    joint_closed_positions=gripper_closed_position,
                    action_deltas=deltas,
                )
            else:
                raise NotImplementedError
        self._attach_gripper = attach_gripper
        return

    @property
    def end_effector(self) -> RigidPrim:
        """[summary]

        Returns:
            RigidPrim: [description]
        """
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        """[summary]

        Returns:
            ParallelGripper: [description]
        """
        return self._gripper

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]
        """
        super().initialize(physics_sim_view)
        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )
        return

    def post_reset(self) -> None:
        """[summary]
        """
        super().post_reset()
        self._gripper.post_reset()
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self.gripper.joint_dof_indicies[0], mode="position"
        )
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self.gripper.joint_dof_indicies[1], mode="position"
        )
        return
