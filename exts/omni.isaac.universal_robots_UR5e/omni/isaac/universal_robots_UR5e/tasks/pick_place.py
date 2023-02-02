# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.isaac.core.tasks as tasks
from omni.isaac.universal_robots_UR5e import UR5e
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.stage import get_stage_units
import numpy as np
from typing import Optional


class PickPlace(tasks.PickPlace):
    """[summary]

        Args:
            name (str, optional): [description]. Defaults to "ur5e_pick_place".
            cube_initial_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            cube_initial_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
            target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            cube_size (Optional[np.ndarray], optional): [description]. Defaults to None.
            offset (Optional[np.ndarray], optional): [description]. Defaults to None.
        """

    def __init__(
        self,
        name: str = "ur5e_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        if cube_size is None:
            cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
        if target_position is None:
            target_position = np.array([0.7, 0.7, cube_size[2] / 2.0])
            target_position[0] = target_position[0] / get_stage_units()
            target_position[1] = target_position[1] / get_stage_units()
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> UR5e:
        """[summary]

        Returns:
            UR5e: [description]
        """
        ur5e_prim_path = find_unique_string_name(
            initial_name="/World/UR5e", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        ur5e_robot_name = find_unique_string_name(
            initial_name="my_ur5e", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        self._ur5e_robot = UR5e(prim_path=ur5e_prim_path, name=ur5e_robot_name, attach_gripper=True,
                                end_effector_prim_name='tool0')
        self._ur5e_robot.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        return self._ur5e_robot

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        tasks.PickPlace.pre_step(self, time_step_index=time_step_index, simulation_time=simulation_time)
        self._ur5e_robot.gripper.update()
        return
