"""
Follow Target Demo with Forward Kinematics Verification

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
Implement robot end-effector following with real-time forward kinematics verification.
The robot tracks a moving target cube using inverse kinematics, while simultaneously
computing forward kinematics to verify the end-effector pose against both the target
and the robot API pose. This provides real-time validation of both IK and FK accuracy.

Features:
- Real-time target tracking using inverse kinematics
- Forward kinematics computation from joint positions
- Pose comparison: FK vs API (actual robot pose)
- Pose comparison: FK vs Target (tracking accuracy)
- Periodic verification output (1 second interval)

reference:
 https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html

Usage:
    # Use default robot type (gbt-c5a) without gripper
    python follow_target_with_fk_verification.py

    # Specify robot type
    python follow_target_with_fk_verification.py --robot_type gbt-c7a
    python follow_target_with_fk_verification.py --robot_type gbt-c12a
    python follow_target_with_fk_verification.py --robot_type gbt-c16a

    # Use gripper
    python follow_target_with_fk_verification.py --robot_type gbt-c5a --attach_gripper

    # View help information
    python follow_target_with_fk_verification.py --help

Arguments:
    --robot_type: Robot type, options: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a (default: gbt-c5a)
    --attach_gripper: Whether to attach gripper to robot (default: False)

Output Format:
    [Frame XXXXX] Target: [x, y, z] | FK: [x, y, z] | API: [x, y, z] | FK-API Error: X.XXXXXX m | FK-Target Error: X.XXXXXX m

"""
import argparse
import os
from typing import Tuple
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.agilebot.kinematics_solver import KinematicsSolver
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


def initialize_controller(robot, robot_type: str, attach_gripper: bool) -> KinematicsSolver:
    """
    Initialize Kinematics Solver (Inverse Kinematics)

    Args:
        robot: Robot object
        robot_type (str): Robot type (gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a)
        attach_gripper (bool): Whether to attach gripper

    Returns:
        KinematicsSolver: Initialized Kinematics Solver instance
    """
    ik_solver = KinematicsSolver(
        robot_articulation=robot,
        robot_type=robot_type,
        attach_gripper=attach_gripper
    )
    return ik_solver


def compute_forward_kinematics(ik_solver: KinematicsSolver, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute forward kinematics from joint positions

    Args:
        ik_solver: Kinematics Solver instance
        joint_positions: Joint positions array

    Returns:
        Tuple[np.ndarray, np.ndarray]: (position, orientation)
            - position: Computed end-effector position
            - orientation: Computed end-effector orientation
    """
    position, orientation = ik_solver._kinematics.compute_forward_kinematics(
        frame_name=ik_solver.get_end_effector_frame(),
        joint_positions=joint_positions,
        position_only=False
    )
    return position, orientation


def get_robot_end_effector_pose(robot, frame_name: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get actual end-effector pose from robot API

    Args:
        robot: Robot object
        frame_name: End-effector frame name (not used, kept for compatibility)

    Returns:
        Tuple[np.ndarray, np.ndarray]: (position, orientation)
            - position: Actual end-effector position
            - orientation: Actual end-effector orientation
    """
    end_effector_prim = robot.end_effector
    pose = end_effector_prim.get_world_pose()
    position = np.array(pose[0])
    orientation = np.array(pose[1])
    return position, orientation


def compute_position_error(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """
    Compute Euclidean distance between two positions

    Args:
        pos1: First position array [x, y, z]
        pos2: Second position array [x, y, z]

    Returns:
        float: Euclidean distance
    """
    return np.linalg.norm(pos1 - pos2)


def main() -> None:
    """
    Main function: Execute follow target demo

    Flow:
    1. Parse command line arguments
    2. Initialize simulation world and task
    3. Get task related objects
    4. Initialize IK solver
    5. Run simulation loop, robot end-effector follows target cube movement
    6. Compute forward kinematics and compare with API pose in real-time
    """
    args = parse_arguments()

    world, task, task_name = initialize_world_and_task(args.robot_type, args.attach_gripper)
    robot, articulation_controller, target_name = get_task_objects(world, task_name)
    ik_solver = initialize_controller(robot, args.robot_type, args.attach_gripper)

    print(f"\n{'='*60}")
    print(f"Follow Target Demo with Forward Kinematics Verification")
    print(f"{'='*60}")
    print(f"Robot Type: {args.robot_type}")
    print(f"End Effector Frame: {ik_solver.get_end_effector_frame()}")
    print(f"Following target: {target_name}")
    print(f"Real-time FK verification enabled")
    print(f"{'='*60}\n")

    physics_dt = world.get_physics_dt()
    accumulated_time = 0.0
    verification_interval = 1.0
    frame_count = 0

    while simulation_app.is_running():
        world.step(render=True)

        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
                continue

            accumulated_time += physics_dt
            frame_count += 1

            observations = task.get_observations()
            actions, success = ik_solver.compute_inverse_kinematics(
                target_position=observations[target_name]['position'],
                target_orientation=observations[target_name]['orientation'],
            )

            if success:
                articulation_controller.apply_action(actions)

                if accumulated_time >= verification_interval:
                    accumulated_time = 0.0
                    joint_positions = actions.joint_positions
                    
                    fk_position, fk_orientation = compute_forward_kinematics(ik_solver, joint_positions)
                    api_position, api_orientation = get_robot_end_effector_pose(robot, ik_solver.get_end_effector_frame())
                    
                    fk_api_error = compute_position_error(fk_position, api_position)
                    target_position = observations[target_name]['position']
                    fk_target_error = compute_position_error(fk_position, target_position)
                    
                    print(f"[Frame {frame_count:5d}] Target: {np.array(target_position).round(4)} | "
                          f"FK: {fk_position.round(4)} | API: {api_position.round(4)} | "
                          f"FK-API Error: {fk_api_error:.6f} m | FK-Target Error: {fk_target_error:.6f} m")
            else:
                print(f"[Frame {frame_count:5d}] IK solver failed to find a solution")

    simulation_app.close()


if __name__ == "__main__":
    main()
