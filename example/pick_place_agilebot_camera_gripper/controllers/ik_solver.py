"""
Copyright © 2016 Agilebot Robotics Ltd. All rights reserved.

Inverse Kinematics Solver Module

This module implements a kinematics solver for computing inverse kinematics
solutions for the robot manipulator.
"""

import os
from typing import Optional

from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)


class KinematicsSolver(ArticulationKinematicsSolver):
    """
    Inverse kinematics solver for the robot manipulator.

    This class provides IK solving capabilities using the Lula kinematics
    solver from Isaac Sim, configured for the GBT C5A robot.
    """

    def __init__(
        self,
        robot_articulation: Articulation,
        end_effector_prim_name: Optional[str] = None
    ) -> None:
        """
        Initialize the kinematics solver.

        Args:
            robot_articulation: Robot articulation object.
            end_effector_prim_name: Name of the end effector prim.
                                   Defaults to "robotiq_arg2f_base_link".
        """
        # Initialize Lula kinematics solver with robot description
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=os.path.join(
                os.path.dirname(__file__),
                "../rmpflow/gbt_c5a_camera_gripper_robot_description.yaml"
            ),
            urdf_path=os.path.join(
                os.path.dirname(__file__),
                "../rmpflow/gbt-c5a_camera_gripper.urdf"
            ),
        )

        # Set default end effector frame if not provided
        if end_effector_prim_name is None:
            end_effector_prim_name = "robotiq_arg2f_base_link"

        # Initialize parent class
        ArticulationKinematicsSolver.__init__(
            self,
            robot_articulation,
            self._kinematics,
            end_effector_prim_name
        )
