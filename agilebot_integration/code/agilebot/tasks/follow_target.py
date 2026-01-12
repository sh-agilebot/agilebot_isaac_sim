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
from typing import Optional

import numpy as np
import isaacsim.core.api.tasks as tasks
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.agilebot import Agilebot

DEFAULT_TARGET_ORIENTATION = np.array([0, 0, 0])


class FollowTarget(tasks.FollowTarget):
    """Follow target task for the Agilebot robot.

    This task makes the robot follow and reach a target position.

    Args:
        robot_usd_path (str): The path to the robot USD file.
        name (str, optional): The name of the task. Defaults to "gbt_follow_target".
        target_prim_path (Optional[str], optional): The path to the target prim. Defaults to None.
            Example: "/World/target".
        target_name (Optional[str], optional): The name of the target cube. Defaults to None.
        target_position (Optional[np.ndarray], optional): The target position of cube. Defaults to None.
        target_orientation (Optional[np.ndarray], optional): The target orientation of cube. Defaults to None.
        offset (Optional[np.ndarray], optional): The offset of the target. Defaults to None.
        prim_path (Optional[str], optional): The path to the GBT prim. Defaults to None.
            Example: "/World/gbt".
        robot_name (Optional[str], optional): The name of the robot. Defaults to None.
        attach_gripper (bool, optional): Whether to attach the gripper to the robot. Defaults to False.
    """

    def __init__(
        self,
        robot_usd_path: str,
        name: str = "agilebot_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        prim_path: Optional[str] = None,
        robot_name: Optional[str] = None,
        attach_gripper: bool = False,
    ) -> None:
        if target_orientation is None:
            target_orientation = euler_angles_to_quat(np.array([0, 0, 0]))

        super().__init__(
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )

        self._prim_path = prim_path
        self._robot_name = robot_name.replace("-", "_")
        self._attach_gripper = attach_gripper
        self._robot_usd_path = robot_usd_path

    def set_robot(self) -> Agilebot:
        """Set and return the robot instance.

        Creates the Agilebot robot instance if it doesn't exist.

        Returns:
            Agilebot: The robot instance.
        """
        if self._robot_name is None:
            self._robot_name = find_unique_string_name(
                initial_name=self._robot_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        if self._prim_path is None:
            self._prim_path = find_unique_string_name(
                initial_name=f"/World/{self._robot_name}",
                is_unique_fn=lambda x: not is_prim_path_valid(x),
            )

        self._agilebot_robot = Agilebot(
            prim_path=self._prim_path,
            name=self._robot_name,
            usd_path=self._robot_usd_path,
            attach_gripper=self._attach_gripper,
        )

        return self._agilebot_robot
