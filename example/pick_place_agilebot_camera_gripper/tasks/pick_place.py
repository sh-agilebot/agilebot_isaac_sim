# flake8: noqa

"""
Copyright © 2016 Agilebot Robotics Ltd. All rights reserved.

Pick and Place Task Module

This module implements the PickPlaceTask class which defines the pick and
place task scenario including the robot, target object, and goal position.

Reference:
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html
"""

import os
from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator


class PickPlaceTask(tasks.PickPlace):
    """
    Pick and place task definition for the GBT C5A robot.

    This class sets up the simulation scene with the robot, target object,
    and goal position for pick and place operations.
    """

    def __init__(
        self,
        name: str = "pick_place_task",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = np.array([0.0515, 0.0515, 0.0515]),
        usd_path: Optional[str] = None,
    ) -> None:
        """
        Initialize the pick and place task.

        Args:
            name: Task name. Defaults to "pick_place_task".
            cube_initial_position: Initial position of the target cube.
            cube_initial_orientation: Initial orientation of the target cube.
            target_position: Goal position for placing the cube.
            offset: Offset for end effector approach.
            cube_size: Dimensions of the target cube (x, y, z).
            usd_path: Path to the robot USD asset file.
        """
        # Initialize parent class
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            offset=offset,
            cube_size=cube_size,
        )

        # Root prim path must contain the robot in the world
        self.root_prim_path = "/World"
        self._usd_path = usd_path
        self.end_effector_prim_path = f"{self.root_prim_path}/gbt_c5a_camera_gripper/robotiq_arg2f_base_link"
        return

    def set_robot(self) -> SingleManipulator:
        """
        Set up the robot manipulator in the scene.

        Returns:
            SingleManipulator: The configured robot manipulator instance.
        """
        # Use provided USD path or fall back to default
        if self._usd_path is not None:
            asset_path = self._usd_path
        else:
            asset_root_path = os.path.join(os.path.dirname(__file__), "../usd")
            if asset_root_path is None:
                raise Exception("Could not find Isaac Sim assets folder")
            asset_path = os.path.join(
                asset_root_path,
                "gbt-c5a_camera_gripper/gbt-c5a_camera_gripper.usd"
            )

        # Load robot asset into the scene
        if os.path.exists(asset_path):
            add_reference_to_stage(usd_path=asset_path, prim_path=self.root_prim_path)
        else:
            raise Exception(f"Could not find robot asset at {asset_path}")

        # Configure parallel gripper
        gripper = ParallelGripper(
            end_effector_prim_path=self.end_effector_prim_path,
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0]),
            joint_closed_positions=np.array([40]),
            action_deltas=np.array([-40]),
            use_mimic_joints=True,
        )

        # Create manipulator instance
        manipulator = SingleManipulator(
            prim_path=self.root_prim_path,
            name="Agilebot",
            end_effector_prim_path=self.end_effector_prim_path,
            gripper=gripper,
        )
        return manipulator

    def set_up_scene(self, scene: Scene) -> None:
        """
        Set up the simulation scene with ground plane, cube, and robot.

        Args:
            scene: The simulation scene object.
        """
        self._scene = scene
        scene.add_default_ground_plane()

        # Generate unique prim path for the cube
        cube_prim_path = find_unique_string_name(
            initial_name="/World/Cube",
            is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        cube_name = find_unique_string_name(
            initial_name="cube",
            is_unique_fn=lambda x: not self.scene.object_exists(x)
        )

        # Adjust cube initial height
        self._cube_initial_position[-1] = 0.02

        # Add dynamic cube to the scene
        self._cube = scene.add(
            DynamicCuboid(
                name=cube_name,
                position=self._cube_initial_position,
                orientation=self._cube_initial_orientation,
                prim_path=cube_prim_path,
                scale=self._cube_size,
                size=0.7,
                color=np.array([0, 0, 1]),
            )
        )
        self._task_objects[self._cube.name] = self._cube

        # Add robot to the scene
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot

        # Move task objects to their reference frames
        self._move_task_objects_to_their_frame()
        return
