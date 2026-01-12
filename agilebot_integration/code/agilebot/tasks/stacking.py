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
from isaacsim.core.api.tasks import Stacking as BaseStacking
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.agilebot import Agilebot
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial

# Define constants in base units (meters) without relying on stage units at import time
DEFAULT_CUBE_INITIAL_POSITIONS_METERS = np.array([[0.5, 0.5, 0.3], [0.5, -0.5, 0.3]])
DEFAULT_TARGET_POSITION_METERS = np.array([0.7, 0, 0])
DEFAULT_CUBE_SIZE_METERS = np.array([0.0515, 0.0515, 0.0515])
DEFAULT_JOINT_POSITIONS = np.array([0, np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, 0, 0, 0, 0, 0, 0, 0])


class Stacking(BaseStacking):
    """
    Stacking task for agilebot.

    This task inherits from BaseStacking and provides specific configuration
    for the Agilebot robot to perform cube stacking operations.

    Args:
        robot_usd_path (str): Path to the robot USD file.
        robot_type (Optional[str]): Type of the robot (e.g., gbt_c5a, gbt_c7a, gbt_c12a, gbt_c16a).
            Defaults to None, which uses default naming.
        name (str, optional): The name of the task. Defaults to "agilebot_stacking".
        target_position (Optional[np.ndarray], optional): Target position of stacking.
            Defaults to None.
        cube_size (Optional[np.ndarray], optional): Size of cube. Defaults to None.
        offset (Optional[np.ndarray], optional): Offset of cube. Defaults to None.
    """

    def __init__(
        self,
        robot_usd_path: str,
        robot_type:str = None,
        name: str = "agilebot_stacking",
        target_position: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:

        if robot_type is None:
            raise ValueError(
                "robot_type is required. Please provide a valid robot type "
                "(e.g., gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)."
            )

        # Get stage units only when initializing an instance, not at import time
        from isaacsim.core.utils.stage import get_stage_units
        stage_units = get_stage_units()

        # Convert default positions from meters to stage units
        default_cube_initial_positions = DEFAULT_CUBE_INITIAL_POSITIONS_METERS / stage_units
        default_target_position = DEFAULT_TARGET_POSITION_METERS / stage_units
        default_cube_size = DEFAULT_CUBE_SIZE_METERS / stage_units

        if cube_size is None:
            cube_size = default_cube_size

        if target_position is None:
            target_position = default_target_position

        super().__init__(
            name=name,
            cube_initial_positions=default_cube_initial_positions,
            cube_initial_orientations=None,
            stack_target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )

        self._robot_usd_path: str = robot_usd_path
        self._robot_type: str = robot_type

    def set_robot(self) -> Agilebot:
        """
        Set up the robot with gripper configuration.

        This method initializes the Agilebot robot with proper prim paths
        and configures the gripper settings for cube manipulation.

        Returns:
            Agilebot: The configured robot instance.
        """
        base_name = self._robot_type.replace("-", "_")
        initial_prim_path = f"/World/{base_name}"

        agilebot_prim_path = find_unique_string_name(
            initial_name=initial_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
        )

        if self._robot_type is not None:
            agilebot_robot_name = find_unique_string_name(
                initial_name=base_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        else:
            raise ValueError(
                "robot_type is required. Please provide a valid robot type "
                "(e.g., gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)."
            )

        self._robot = Agilebot(
            prim_path=agilebot_prim_path,
            name=agilebot_robot_name,
            usd_path=self._robot_usd_path,
            attach_gripper=True,
        )

        self._robot.set_joints_default_state(positions=DEFAULT_JOINT_POSITIONS)

        return self._robot

    def set_up_scene(self, scene) -> None:
        """
        Set up the scene with ground plane and cubes.

        This method initializes the scene by adding a ground plane and creating
        cubes at specified initial positions for stacking operations.

        Args:
            scene: The scene object to configure.
        """
        self._scene = scene
        scene.add_default_ground_plane()

        for i in range(self._num_of_cubes):
            color = np.random.uniform(size=(3,))
            cube_prim_path = find_unique_string_name(
                initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            cube_name = find_unique_string_name(
                initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )

            self._cubes.append(
                scene.add(
                    DynamicCuboid(
                        name=cube_name,
                        position=self._cube_initial_positions[i],
                        orientation=self._cube_initial_orientations[i],
                        prim_path=cube_prim_path,
                        scale=self._cube_size,
                        size=1.0,
                        color=color,
                        mass=0.01,
                        physics_material=PhysicsMaterial(
                            prim_path=f"/World/Physics_Materials/physics_material_{i}",
                            static_friction=1.0,
                            dynamic_friction=1.0,
                            restitution=0.0,
                        ),
                    )
                )
            )
            self._task_objects[self._cubes[-1].name] = self._cubes[-1]

        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
