"""
Copyright © 2025 Agilebot Robotics Ltd. All rights reserved.
Author: Desheng.Li, September 4, 2025
Instruction:
Implement pick and place functionality following Isaac Sim official tutorial. This file is the controller module that implements a PickPlaceController class for controlling robot pick and place actions.
reference:
 https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html

"""

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers import ParallelGripper
from controllers.rmpflow_controller import RMPFlowController


class PickPlaceController(manipulators_controllers.PickPlaceController):
    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        events_dt=None,
        end_effector_initial_height: float | None = None,
    ) -> None:
        if events_dt is None:
            events_dt = [
                0.005,
                0.002,
                1,
                0.05,
                0.0008,
                0.005,
                0.0008,
                0.1,
                0.0008,
                0.008,
            ]
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
