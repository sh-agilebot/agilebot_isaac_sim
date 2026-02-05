"""
Copyright © 2016 Agilebot Robotics Ltd. All rights reserved.

Pick and Place Controller Module

This module implements the PickPlaceController class which controls the robot's
pick and place actions using the RMPFlow motion planning algorithm.

Reference:
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html
"""

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers import ParallelGripper
from controllers.rmpflow_controller import RMPFlowController


class PickPlaceController(manipulators_controllers.PickPlaceController):
    """
    Controller for pick and place operations using RMPFlow motion planning.

    This controller extends the Isaac Sim PickPlaceController to provide
    pick and place functionality with configurable timing for each stage
    of the manipulation task.
    """

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        events_dt=None,
        end_effector_initial_height: float | None = None,
    ) -> None:
        """
        Initialize the pick and place controller.

        Args:
            name: Name of the controller.
            gripper: Parallel gripper instance for end effector control.
            robot_articulation: Robot articulation object.
            events_dt: Time deltas for each stage of the pick and place task.
                       Defaults to a predefined sequence of timing values.
            end_effector_initial_height: Initial height of the end effector.
        """
        if events_dt is None:
            # Default timing configuration for pick and place stages
            events_dt = [
                0.005,   # Approach pick
                0.002,   # Lift after pick
                1,       # Hold at pick
                0.05,    # Move to place
                0.0008,  # Approach place
                0.005,   # Lift after place
                0.0008,  # Retreat from place
                0.1,     # Hold at place
                0.0008,  # Final retreat
                0.008,   # End of task
            ]

        # Initialize parent class with RMPFlow controller for motion planning
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller",
                robot_articulation=robot_articulation,
            ),
            gripper=gripper,
            events_dt=events_dt,
            end_effector_initial_height=end_effector_initial_height,
        )
        return
