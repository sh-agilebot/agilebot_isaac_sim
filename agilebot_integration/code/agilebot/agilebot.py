# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List, Optional

import carb
import numpy as np
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

DEFAULT_GRIPPER_OPEN_POSITION = np.array([0])
DEFAULT_GRIPPER_CLOSED_POSITION = np.array([40])
DEFAULT_GRIPPER_DELTAS = np.array([-40])
DEFAULT_GRIPPER_DOF_NAMES = ["finger_joint"]


class Agilebot(Robot):
    """
    The Agilebot is a 6-DOF robot with a gripper that can be attached to the end-effector.

    Args:
        prim_path (str): The path to the robot in the USD stage.
        name (str, optional): Name of the robot. Defaults to "agilebot_robot".
        usd_path (Optional[str], optional): The USD path of the robot. Defaults to None.
        position (Optional[np.ndarray], optional): The position of the robot. Defaults to None.
        orientation (Optional[np.ndarray], optional): The orientation of the robot. Defaults to None.
        end_effector_prim_name (Optional[str], optional): The name of the end-effector prim. Defaults to None.
        gripper_dof_names (Optional[List[str]], optional): Names of the gripper's DOFs. Defaults to None.
        gripper_open_position (Optional[np.ndarray], optional): The position of the gripper when open. Defaults to None.
        gripper_closed_position (Optional[np.ndarray], optional): The position of the gripper when closed. Defaults to None.
        deltas (Optional[np.ndarray], optional): Delta values for gripper. Defaults to None.
        attach_gripper (bool, optional): Whether to attach a gripper to the robot. Defaults to False.

    Raises:
        FileNotFoundError: If prim_path is not valid and usd_path is not provided.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "agilebot_robot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
        gripper_dof_names: Optional[List[str]] = None,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
        deltas: Optional[np.ndarray] = None,
        attach_gripper: bool = False,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name

        if not prim.IsValid():
            if usd_path:
                _robot = add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
                carb.log_info(f"usd_path: {usd_path}")

                if attach_gripper:
                    _robot.GetVariantSet("Gripper").SetVariantSelection("robotiq_2f_140")
                else:
                    _robot.GetVariantSet("Gripper").SetVariantSelection("None")
            else:
                raise FileNotFoundError("usd_path is required if prim_path is not valid")

        if attach_gripper:
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/ee_link/robotiq_arg2f_base_link"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name

            if gripper_dof_names is None:
                gripper_dof_names = DEFAULT_GRIPPER_DOF_NAMES
            if gripper_open_position is None:
                gripper_open_position = DEFAULT_GRIPPER_OPEN_POSITION
            if gripper_closed_position is None:
                gripper_closed_position = DEFAULT_GRIPPER_CLOSED_POSITION
            if deltas is None:
                deltas = DEFAULT_GRIPPER_DELTAS
        else:
            self._end_effector_prim_path = prim_path + "/link6"

        super().__init__(
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
            articulation_controller=None,
        )

        if attach_gripper:
            self._gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_position,
                joint_closed_positions=gripper_closed_position,
                action_deltas=deltas,
                use_mimic_joints=True,
            )
        else:
            self._gripper = None

    @property
    def end_effector(self) -> SingleRigidPrim:
        """Get the end effector rigid prim.

        Returns:
            SingleRigidPrim: The end effector rigid prim instance.
        """
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        """Get the gripper instance.

        Returns:
            ParallelGripper: The gripper instance attached to the robot.
        """
        return self._gripper

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize the robot, end effector, and gripper.

        Args:
            physics_sim_view: Optional physics simulation view.
        """
        super().initialize(physics_sim_view)
        self._end_effector = SingleRigidPrim(
            prim_path=self._end_effector_prim_path, name=self.name + "_end_effector"
        )
        self._end_effector.initialize(physics_sim_view)

        if self._gripper is not None:
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )

        self.disable_gravity()

    def post_reset(self) -> None:
        """Reset the robot and gripper to initial state."""
        super().post_reset()

        if self._gripper is not None:
            self._gripper.post_reset()
            self._articulation_controller.switch_dof_control_mode(
                dof_index=self.gripper.joint_dof_indicies[0], mode="position"
            )
        # if you have 2-dofs for the gripper,pleade set the second dof to position mode
        # self._articulation_controller.switch_dof_control_mode(
        #     dof_index=self.gripper.joint_dof_indicies[1], mode="position"
        # )
        return
