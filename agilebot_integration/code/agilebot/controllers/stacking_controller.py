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

import numpy as np
import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.agilebot.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper


class StackingController(manipulators_controllers.StackingController):
    """Controller for stacking cubes.

    This controller manages the robot's motion to pick up cubes and stack them on top of each other
    in a specified order. It uses the PickPlaceController for individual pick and place operations.

    Args:
        name: Name of the controller.
        gripper: The parallel gripper to use for grasping cubes.
        robot_articulation: The robot articulation to control.
        picking_order_cube_names: The order of the cubes to pick (list of cube names).
        robot_observation_name: The name of the robot observation (should be robot.name).
        robot_type: Type of the robot (e.g., gbt_c5a, gbt_c7a, gbt_c12a, gbt_c16a).
    """

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        picking_order_cube_names: List[str],
        robot_observation_name: str,
        robot_type: str,
    ) -> None:
        super().__init__(
            name=name,
            pick_place_controller=PickPlaceController(
                name=name + "_pick_place_controller",
                gripper=gripper,
                robot_articulation=robot_articulation,
                robot_type=robot_type,
            ),
            picking_order_cube_names=picking_order_cube_names,
            robot_observation_name=robot_observation_name,
        )

    def forward(
        self,
        observations: dict,
        end_effector_orientation: Optional[np.ndarray] = None,
        end_effector_offset: Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """Forward pass of the controller.

        Args:
            observations (dict): Dictionary containing robot and cube observations.
            end_effector_orientation (Optional[np.ndarray], optional): End effector orientation while picking and placing. Defaults to None.
            end_effector_offset (Optional[np.ndarray], optional): Offset of the end effector target. Defaults to None.

        Returns:
            ArticulationAction: The action to take.
        """
        return super().forward(
            observations,
            end_effector_orientation=end_effector_orientation,
            end_effector_offset=end_effector_offset,
        )
