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
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.agilebot import Agilebot

DEFAULT_ROBOT_TYPE = "gbt-c5a"


class PickPlace(tasks.PickPlace):
    """Task class for pick and place operations with the Agilebot robot.

    This task manages the robot's motion to pick up a cube and place it at a target location.

    Args:
        usd_path (str): Path to the USD file containing the robot model.
        name (str, optional): Name of the task. Defaults to "agilebot_pick_place".
        cube_initial_position (Optional[np.ndarray], optional): Initial position of the cube. Defaults to None.
        cube_initial_orientation (Optional[np.ndarray], optional): Initial orientation of the cube. Defaults to None.
        target_position (Optional[np.ndarray], optional): Target position where the cube should be placed. Defaults to None.
        cube_size (Optional[np.ndarray], optional): Size of the cube. Defaults to None.
        offset (Optional[np.ndarray], optional): Offset of the cube. Defaults to None.
        robot_type (str, optional): Type of the robot. Defaults to "gbt-c5a".
            Examples: "gbt-c5a", "gbt-c7a", "gbt-c12a", "gbt-c16a".
    """

    def __init__(
        self,
        usd_path: str,
        name: str = "agilebot_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        robot_type: str = DEFAULT_ROBOT_TYPE,
    ) -> None:
        super().__init__(
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )

        self._usd_path = usd_path
        self._robot_type = robot_type

    def set_robot(self) -> Agilebot:
        """Set and return the robot instance.

        Creates the Agilebot robot instance if it doesn't exist.

        Returns:
            Agilebot: The robot instance.
        """
        agilebot_prim_path = find_unique_string_name(
            initial_name=f"/World/{self._robot_type.replace('-', '_')}", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        agilebot_robot_name = find_unique_string_name(
            initial_name=self._robot_type.replace("-", "_"), is_unique_fn=lambda x: not self.scene.object_exists(x)
        )

        return Agilebot(
            prim_path=agilebot_prim_path,
            name=agilebot_robot_name,
            usd_path=self._usd_path,
            attach_gripper=True,
        )
