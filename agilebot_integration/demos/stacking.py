"""
Stacking Demo - Robot stacks cubes

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
Implement stacking functionality based on Isaac Sim official tutorial. This file is the entry point
that launches a standalone Isaac Sim application and loads a stacking task.


Usage:
    # Use default robot type (gbt-c5a)
    python stacking.py

    # Specify robot type
    python stacking.py --robot_type gbt-c7a
    python stacking.py --robot_type gbt-c12a
    python stacking.py --robot_type gbt-c16a

    # Set maximum gripper force in Newtons
    python stacking.py --max_force 3.0

    # View help information
    python stacking.py --help

Arguments:
    --robot_type: Robot type, options: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a (default: gbt-c5a)
    --max_force: Maximum gripper force in Newtons (default: 2.0)

"""
import argparse
import os
from typing import Tuple
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.agilebot.controllers.stacking_controller import StackingController
from isaacsim.robot.manipulators.examples.agilebot.tasks import Stacking


def parse_arguments() -> argparse.Namespace:
    """
    Parse command line arguments

    Returns:
        argparse.Namespace: Parsed command line arguments
            - robot_type: Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)
            - max_force: Maximum gripper force in Newtons
    """
    parser = argparse.ArgumentParser(description="Stacking Example with configurable robot type")
    parser.add_argument("--robot_type", type=str, default="gbt-c5a",
                        choices=["gbt-c5a", "gbt-c7a", "gbt-c12a", "gbt-c16a"],
                        help="Robot type to use (default: gbt-c5a)")
    parser.add_argument("--max_force", type=float, default=2.0,
                        help="Maximum gripper force in Newtons (default: 2.0)")
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


def initialize_world_and_task(robot_type: str) -> Tuple[World, Stacking, str, dict]:
    """
    Initialize simulation world and stacking task

    Args:
        robot_type (str): Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)

    Returns:
        Tuple[World, Stacking, str, dict]: Tuple containing world instance, task instance, task name, and task params

    Raises:
        FileNotFoundError: When USD file does not exist
    """
    usd_path = f"/home/gbt/ws/usd/{robot_type}/{robot_type}.usd"
    task_name = f"{robot_type.replace('-', '_')}_stacking_task"

    validate_usd_path(usd_path)

    world = World(stage_units_in_meters=1.0)

    task = Stacking(
        robot_usd_path=usd_path,
        name=task_name,
        robot_type=robot_type
    )

    world.add_task(task)
    world.reset()

    task_params = world.get_task(task_name).get_params()

    return world, task, task_name, task_params


def get_robot_objects(world: World, task_name: str, task_params: dict) -> Tuple:
    """
    Get robot object and controller from task

    Args:
        world (World): Simulation world instance
        task_name (str): Task name
        task_params (dict): Task parameters

    Returns:
        Tuple: Tuple containing robot object and articulation controller
            - robot: Robot object
            - articulation_controller: Articulation controller
    """
    robot_name = task_params["robot_name"]["value"]
    robot = world.scene.get_object(robot_name)
    articulation_controller = robot.get_articulation_controller()

    return robot, articulation_controller


def initialize_controller(robot, robot_type: str, cube_names: list) -> StackingController:
    """
    Initialize Stacking controller

    Args:
        robot: Robot object
        robot_type (str): Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)
        cube_names (list): List of cube names for stacking order

    Returns:
        StackingController: Initialized Stacking controller instance
    """
    controller = StackingController(
        name="stacking_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        picking_order_cube_names=cube_names,
        robot_observation_name=robot.name,
        robot_type=robot_type
    )
    controller.reset()
    return controller


def set_gripper_max_force(robot, articulation_controller, max_force: float = 2.0) -> None:
    """
    Dynamically set the maximum force for the gripper

    Args:
        robot: Robot object
        articulation_controller: Articulation controller
        max_force (float): Maximum force value in Newtons (default: 2.0)
    """
    gripper_joint_indices = robot.gripper.active_joint_indices
    
    if gripper_joint_indices:
        max_force_values = [max_force] * len(gripper_joint_indices)
        max_force_array = np.array(max_force_values)
        
        current_max_efforts = articulation_controller.get_max_efforts()
        print(f"Current max efforts before setting: {current_max_efforts}")
        
        robot._articulation_view.set_max_efforts(max_force_array, joint_indices=gripper_joint_indices)
        
        new_max_efforts = articulation_controller.get_max_efforts()
        print(f"Current max efforts after setting: {new_max_efforts}")
        
        for joint_idx in gripper_joint_indices:
            joint_name = robot.dof_names[joint_idx]
            print(f"Joint {joint_name} max effort set to: {max_force} N")


def main() -> None:
    """
    Main function: Execute stacking demo

    Flow:
        1. Parse command line arguments
        2. Initialize simulation world and task
        3. Get robot objects
        4. Initialize controller
        5. Run simulation loop, robot stacks cubes
    """
    args = parse_arguments()

    world, task, task_name, task_params = initialize_world_and_task(args.robot_type)
    robot, articulation_controller = get_robot_objects(world, task_name, task_params)
    controller = initialize_controller(robot, args.robot_type, task.get_cube_names())

    reset_needed = False
    task_completed = False
    max_force_set = False

    while simulation_app.is_running():
        world.step(render=True)

        if world.is_playing():
            if not max_force_set:
                set_gripper_max_force(robot, articulation_controller, max_force=args.max_force)
                max_force_set = True

            if reset_needed:
                world.reset()
                reset_needed = False
                controller.reset()
                task_completed = False

            if world.current_time_step_index == 0:
                controller.reset()

            observations = world.get_observations()
            actions = controller.forward(observations=observations)

            if controller.is_done() and not task_completed:
                print("Done stacking")
                task_completed = True

            articulation_controller.apply_action(actions)

        if world.is_stopped():
            reset_needed = True

    simulation_app.close()


if __name__ == "__main__":
    main()
