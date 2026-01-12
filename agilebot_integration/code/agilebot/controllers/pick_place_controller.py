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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.agilebot.controllers.rmpflow_controller import RMPFlowController
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
import isaacsim.robot.manipulators.controllers as manipulators_controllers

DEFAULT_EVENTS_DT = [0.01, 0.0035, 0.01, 1.0, 0.008, 0.005, 0.005, 1, 0.01, 0.08]

PHASE_DESCRIPTIONS = [
    "Move end_effector above the cube center at the 'end_effector_initial_height'.",
    "Lower end_effector down to encircle the target cube.",
    "Wait for Robot's inertia to settle.",
    "Close grip.",
    "Move end_effector up again, keeping the grip tight (lifting the block).",
    "Smoothly move the end_effector toward the goal xy, keeping the height constant.",
    "Move end_effector vertically toward goal height at the 'end_effector_initial_height'.",
    "Loosen the grip.",
    "Move end_effector vertically up again at the 'end_effector_initial_height'.",
    "Move end_effector towards the old xy position.",
]


class PickPlaceController(manipulators_controllers.PickPlaceController):
    """The PickPlaceController is a controller that is used to pick and place a cube.

    Args:
        name: Name of the controller.
        gripper: The parallel gripper to use for grasping objects.
        robot_articulation: The robot articulation to control.
        robot_type: Type of the robot (e.g., gbt_c5a, gbt_c7a, gbt_c12a, gbt_c16a).
        events_dt: Time duration for each phase of the pick and place operation. 10 phases must be defined.
            Defaults to None, which uses predefined values.
        end_effector_initial_height: Initial height of the end effector. Defaults to None.

    Important Warning:
        If your robot's base is elevated above ground level, the specified height must be set greater than the base height.
        For example, if the work platform height is 1m, this value should be configured to 1.2m (or higher).
        Otherwise, the robot's end-effector may crash directly into the ground.
    """

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        robot_type: str,
        events_dt: Optional[List[float]] = None,
        end_effector_initial_height: Optional[float] = None,
    ) -> None:
        if events_dt is None:
            events_dt = DEFAULT_EVENTS_DT

        super().__init__(
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller",
                robot_articulation=robot_articulation,
                attach_gripper=True,
                robot_type=robot_type,
            ),
            gripper=gripper,
            events_dt=events_dt,
            end_effector_initial_height=end_effector_initial_height,
        )

    def forward(
        self,
        picking_position: np.ndarray,
        placing_position: np.ndarray,
        current_joint_positions: np.ndarray,
        end_effector_offset: Optional[np.ndarray] = None,
        end_effector_orientation: Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """Forward pass of the controller.

        Args:
            picking_position (np.ndarray): Position of the cube to pick.
            placing_position (np.ndarray): Position of the cube to place.
            current_joint_positions (np.ndarray): Current joint positions of the robot.
            end_effector_offset (Optional[np.ndarray], optional): Offset of the end effector target. Defaults to None.
            end_effector_orientation (Optional[np.ndarray], optional): End effector orientation while picking and placing. Defaults to None.

        Returns:
            ArticulationAction: The action to take.
        """
        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi,0])) # the orientation of the end_effector
            
        if end_effector_offset is None:
            end_effector_offset = np.array([0, 0, 0.20])  # the offset of the end_effector
        return super().forward(
            picking_position,
            placing_position,
            current_joint_positions,
            end_effector_offset=end_effector_offset,
            end_effector_orientation=end_effector_orientation,
        )
