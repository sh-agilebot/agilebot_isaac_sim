"""
Follow Target Demo - Robot end-effector follows cube movement

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Instruction:
Implement gripper following based on Isaac Sim official tutorial. This file is the entry point
that launches a standalone Isaac Sim application.
reference:
 https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html

Usage:
    # Use default robot type (gbt-c5a) without gripper
    python follow_target.py

    # Specify robot type
    python follow_target.py --robot_type gbt-c7a
    python follow_target.py --robot_type gbt-c12a
    python follow_target.py --robot_type gbt-c16a

    # Use gripper
    python follow_target.py --robot_type gbt-c5a --attach_gripper

    # View help information
    python follow_target.py --help

Arguments:
    --robot_type: Robot type, options: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a (default: gbt-c5a)
    --attach_gripper: Whether to attach gripper to robot (default: False)

"""
import argparse
import os
from typing import Tuple
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.agilebot.controllers.rmpflow_controller import RMPFlowController
from isaacsim.robot.manipulators.examples.agilebot.tasks.follow_target import FollowTarget


def parse_arguments() -> argparse.Namespace:
    """
    Parse command line arguments
    
    Returns:
        argparse.Namespace: Parsed command line arguments
            - robot_type: Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)
            - attach_gripper: Whether to attach gripper
    """
    parser = argparse.ArgumentParser(description="Follow Target Example with configurable robot type")
    parser.add_argument("--robot_type", type=str, default="gbt-c5a",
                        choices=["gbt-c5a", "gbt-c7a", "gbt-c12a", "gbt-c16a"],
                        help="Robot type to use (default: gbt-c5a)")
    parser.add_argument("--attach_gripper", action="store_true",
                        help="Attach gripper to the robot (default: False)")
    return parser.parse_args()


def validate_usd_path(usd_path: str) -> str:
    """
    Validate USD file path existence
    
    Args:
        usd_path (str): USD file path
    
    Returns:
        str: Validated USD file path
    
    Raises:
        FileNotFoundError: When USD file does not exist
    """
    if not os.path.exists(usd_path):
        raise FileNotFoundError(f"USD file not found: {usd_path}")
    return usd_path


def initialize_world_and_task(robot_type: str, attach_gripper: bool) -> Tuple[World, FollowTarget, str]:
    """
    Initialize simulation world and follow target task
    
    Args:
        robot_type (str): Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)
        attach_gripper (bool): Whether to attach gripper to robot
    
    Returns:
        Tuple[World, FollowTarget, str]: Tuple containing world instance, task instance, and task name
    
    Raises:
        FileNotFoundError: When USD file does not exist
    """
    robot_type_underscore = robot_type.replace('-', '_')
    task_name = f"{robot_type_underscore}_follow_target_task"
    usd_path = f"/home/gbt/ws/usd/{robot_type}/{robot_type}.usd"
    
    validate_usd_path(usd_path)
    
    world = World(stage_units_in_meters=1.0)
    
    task = FollowTarget(
        robot_usd_path=usd_path,
        name=task_name,
        target_position=np.array([0.5, 0, 0.5]),
        attach_gripper=attach_gripper,
        robot_name=robot_type_underscore
    )
    
    world.add_task(task)
    world.reset()
    
    return world, task, task_name


def get_task_objects(world: World, task_name: str) -> Tuple:
    """
    Get robot object and controller from task
    
    Args:
        world (World): Simulation world instance
        task_name (str): Task name
    
    Returns:
        Tuple: Tuple containing robot object, articulation controller, and target name
            - robot: Robot object
            - articulation_controller: Articulation controller
            - target_name: Target name
    """
    task_params = world.get_task(task_name).get_params()
    target_name = task_params["target_name"]['value']
    robot_name = task_params["robot_name"]['value']
    robot = world.scene.get_object(robot_name)
    articulation_controller = robot.get_articulation_controller()
    
    return robot, articulation_controller, target_name


def initialize_controller(robot, robot_type: str, attach_gripper: bool) -> RMPFlowController:
    """
    Initialize RMPFlow controller
    
    Args:
        robot: Robot object
        robot_type (str): Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)
        attach_gripper (bool): Whether to attach gripper
    
    Returns:
        RMPFlowController: Initialized RMPFlow controller instance
    """
    controller = RMPFlowController(
        name="rmpflow_controller",
        robot_articulation=robot,
        robot_type=robot_type,
        attach_gripper=attach_gripper
    )
    controller.reset()
    return controller


def main() -> None:
    """
    Main function: Execute follow target demo
    
    Flow:
    1. Parse command line arguments
    2. Initialize simulation world and task
    3. Get task related objects
    4. Initialize controller
    5. Run simulation loop, robot end-effector follows target cube movement
    """
    args = parse_arguments()
    
    world, task, task_name = initialize_world_and_task(args.robot_type, args.attach_gripper)
    robot, articulation_controller, target_name = get_task_objects(world, task_name)
    controller = initialize_controller(robot, args.robot_type, args.attach_gripper)
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
                controller.reset()

            observations = task.get_observations()
            target_position = observations[target_name]['position']
            target_orientation = observations[target_name]['orientation']
            
            actions = controller.forward(
                target_end_effector_orientation=target_orientation,
                target_end_effector_position=target_position,
            )
            articulation_controller.apply_action(actions)
    
    simulation_app.close()


if __name__ == "__main__":
    main()
