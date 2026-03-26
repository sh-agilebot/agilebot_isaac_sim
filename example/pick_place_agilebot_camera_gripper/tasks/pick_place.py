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
from runtime_config import (
    get_end_effector_prim_name_candidates,
    get_gripper_joint_prim_names,
    get_robot_root_prim_path,
    get_usd_candidate_paths,
)
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, update_stage
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator


def resolve_robot_usd_path(explicit_usd_path: Optional[str] = None) -> str:
    """Resolve the robot USD asset path from known local candidates."""
    if explicit_usd_path is not None:
        return explicit_usd_path

    candidate_paths = get_usd_candidate_paths()
    for candidate_path in candidate_paths:
        if os.path.exists(candidate_path):
            return candidate_path

    raise FileNotFoundError(
        "Could not find robot asset. Checked: "
        + ", ".join(candidate_paths)
    )


def find_descendant_prim_path(root_prim_path: str, prim_name_candidates: list[str]) -> str:
    """Find the first descendant prim whose name matches one of the candidates."""
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"Root prim does not exist: {root_prim_path}")

    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        if not prim_path.startswith(f"{root_prim_path}/"):
            continue
        if prim.GetName() in prim_name_candidates:
            return prim_path

    raise RuntimeError(
        f"Could not find prim under {root_prim_path}. "
        f"Tried names: {prim_name_candidates}"
    )


def find_descendant_camera_prim_path(
    root_prim_path: str, prim_name_candidates: Optional[list[str]] = None
) -> str:
    """Find a descendant Camera prim, optionally preferring specific names."""
    stage = get_current_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"Root prim does not exist: {root_prim_path}")

    camera_candidates: list[str] = []
    fallback_cameras: list[str] = []

    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        if not prim_path.startswith(f"{root_prim_path}/"):
            continue
        if prim.GetTypeName() != "Camera":
            continue
        if prim_name_candidates and prim.GetName() in prim_name_candidates:
            camera_candidates.append(prim_path)
        else:
            fallback_cameras.append(prim_path)

    if camera_candidates:
        return camera_candidates[0]
    if fallback_cameras:
        return fallback_cameras[0]

    raise RuntimeError(
        f"Could not find a Camera prim under {root_prim_path}. "
        f"Preferred names: {prim_name_candidates}"
    )


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

        # Root prim path is where the robot USD reference is inserted.
        self.root_prim_path = get_robot_root_prim_path()
        self._usd_path = resolve_robot_usd_path(usd_path)
        self.end_effector_prim_path = None
        return

    def set_robot(self) -> SingleManipulator:
        """
        Set up the robot manipulator in the scene.

        Returns:
            SingleManipulator: The configured robot manipulator instance.
        """
        # Use provided USD path or fall back to default
        asset_path = self._usd_path

        # Load robot asset into the scene
        if os.path.exists(asset_path):
            add_reference_to_stage(usd_path=asset_path, prim_path=self.root_prim_path)
            update_stage()
        else:
            raise Exception(f"Could not find robot asset at {asset_path}")

        self.end_effector_prim_path = find_descendant_prim_path(
            self.root_prim_path,
            get_end_effector_prim_name_candidates(),
        )

        # Configure parallel gripper
        gripper = ParallelGripper(
            end_effector_prim_path=self.end_effector_prim_path,
            joint_prim_names=get_gripper_joint_prim_names(),
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
