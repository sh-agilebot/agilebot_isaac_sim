"""
Copyright © 2016 Agilebot Robotics Ltd. All rights reserved.

Inverse Kinematics Solver Module

This module implements a kinematics solver for computing inverse kinematics
solutions for the robot manipulator.
"""

import os
from typing import Optional

from isaacsim.core.prims import Articulation
from runtime_config import (
    get_end_effector_frame_name,
    get_robot_description_path,
    get_urdf_path,
)
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
                                   Defaults to the gripper base link in the URDF.
        """
        # Initialize Lula kinematics solver with robot description
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=get_robot_description_path(),
            urdf_path=get_urdf_path(),
        )

        # Set default end effector frame if not provided
        if end_effector_prim_name is None:
            end_effector_prim_name = get_end_effector_frame_name()

        # Initialize parent class
        ArticulationKinematicsSolver.__init__(
            self,
            robot_articulation,
            self._kinematics,
            end_effector_prim_name
        )
