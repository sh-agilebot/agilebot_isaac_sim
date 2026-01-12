"""
Copyright © 2025 Agilebot Robotics Ltd. All rights reserved.
Author: Desheng.Li, September 4, 2025
Instruction:
Implement pick and place functionality following Isaac Sim official tutorial. This file is the task module that implements a PickPlaceTask class for defining pick and place tasks.
reference:
 https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html

"""
# flake8: noqa
import os  # Import OS module for file path handling
from typing import Optional  # Import Optional type for values that may be None
import isaacsim.core.api.tasks as tasks  # Import Isaac Sim task related APIs
import numpy as np  # Import NumPy library for numerical computation
from isaacsim.core.utils.stage import add_reference_to_stage  # Import tool for adding reference to scene
from isaacsim.robot.manipulators.grippers import ParallelGripper  # Import parallel gripper class
from isaacsim.robot.manipulators.manipulators import SingleManipulator  # Import single manipulator class
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.prims import is_prim_path_valid
# from isaacsim.storage.native import get_assets_root_path  # Commented out import for getting assets root path
class PickPlaceTask(tasks.PickPlace):  # Define PickPlaceTask class, inheriting from Isaac Sim's PickPlace task class
    """
        PickPlaceTask class for defining pick and place tasks.
        Args:
            name (str, optional): _description_. Defaults to "pick_place_task".
            cube_initial_position (Optional[np.ndarray], optional): _description_. Defaults to None.
            cube_initial_orientation (Optional[np.ndarray], optional): _description_. Defaults to None.
            target_position (Optional[np.ndarray], optional): _description_. Defaults to None.
            offset (Optional[np.ndarray], optional): _description_. Defaults to None.
            cube_size (Optional[np.ndarray], optional): _description_. Defaults to np.array([0.0515, 0.0515, 0.0515]).
    """
    def __init__(
        self,
        name:str="pick_place_task",  # Task name, default is "pick_place_task" 
        cube_initial_position:Optional[np.ndarray]=None,  # Cube initial position, optional NumPy array
        cube_initial_orientation:Optional[np.ndarray]=None,  # Cube initial orientation, optional NumPy array
        target_position:Optional[np.ndarray]=None,  # Target position, optional NumPy array
        offset:Optional[np.ndarray]=None,  # Offset, optional NumPy array
        cube_size:Optional[np.ndarray]=np.array([0.0515, 0.0515, 0.0515])  # Cube size, default is [0.0515, 0.0515, 0.0515]
    ):

        # Call parent class PickPlace initialization method
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            offset=offset,
            cube_size=cube_size,
        )
        self.root_prim_path="/World" # Must be world containing a robot
        
        self.end_effector_prim_path=f"{self.root_prim_path}/gbt_c5a_camera_gripper/robotiq_arg2f_base_link"
        return
    
    def set_robot(self)->SingleManipulator:  # Set robot and return single manipulator object
        asset_root_path =os.path.join(os.path.dirname(__file__),"../usd")
        if asset_root_path is None:  # Check if assets root path exists
            raise Exception("Could not find Isaac Sim assets folder")
        
        # Build robot gripper asset path
        asset_path=os.path.join(asset_root_path, "gbt_c5a_camera_gripper/gbt_c5a_camera_gripper.usd")

        if os.path.exists(asset_path):  # Check if robot gripper asset file exists
            add_reference_to_stage(usd_path=asset_path, prim_path=self.root_prim_path)
        else:
            raise Exception(f"Could not find robot asset at {asset_path}")


        
        gripper=ParallelGripper(
            end_effector_prim_path=self.end_effector_prim_path,
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0]),
            joint_closed_positions=np.array([40]),
            action_deltas=np.array([-40]),
            use_mimic_joints=True,
        )
        # Instantiate single manipulator object
        manipulator=SingleManipulator(
            prim_path=self.root_prim_path,
            name="Agilebot",
            end_effector_prim_path=self.end_effector_prim_path,
            gripper=gripper,
        )
        return manipulator
    
    
    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        self._scene = scene
        scene.add_default_ground_plane()
        cube_prim_path = find_unique_string_name(
            initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x))
        self._cube_initial_position[-1]=0.02
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
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return