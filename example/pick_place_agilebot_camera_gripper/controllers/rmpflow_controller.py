"""
Copyright © 2016 Agilebot Robotics Ltd. All rights reserved.

RMPFlow Motion Planning Controller Module

This module implements the RMPFlowController class which provides motion
planning and collision detection using the RMPFlow algorithm.

Reference:
    https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html
"""

import os
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import Articulation


class RMPFlowController(mg.MotionPolicyController):
    """
    Motion planning controller using RMPFlow algorithm.

    RMPFlow (Riemannian Motion Policies) is a unified framework for motion
    planning that combines multiple motion policies (e.g., goal-reaching,
    collision avoidance) into a single accelerations command.
    """

    def __init__(
        self,
        name: str,
        robot_articulation: Articulation,
        physical_dt: float = 1.0 / 60.0
    ) -> None:
        """
        Initialize the RMPFlow controller.

        Args:
            name: Name of the controller.
            robot_articulation: Robot articulation object.
            physical_dt: Physics timestep in seconds. Defaults to 1/60.
        """
        # Initialize RMPFlow motion policy with robot description files
        self.rmpflow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=os.path.join(
                os.path.dirname(__file__),
                "../rmpflow/gbt_c5a_camera_gripper_robot_description.yaml"
            ),
            urdf_path=os.path.join(
                os.path.dirname(__file__),
                "../rmpflow/gbt-c5a_camera_gripper.urdf"
            ),
            rmpflow_config_path=os.path.join(
                os.path.dirname(__file__),
                "../rmpflow/gbt_c5a_rmpflow_common.yaml"
            ),
            end_effector_frame_name="robotiq_arg2f_base_link",
            maximum_substep_size=0.00334,
        )

        # Create articulation motion policy interface
        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation=robot_articulation,
            motion_policy=self.rmpflow,
            default_physics_dt=physical_dt,
        )

        # Initialize parent class
        mg.MotionPolicyController.__init__(self, name, self.articulation_rmp)

        # Store and set robot base pose
        self._defalut_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            self._defalut_position, self._default_orientation
        )

        return

    def reset(self):
        """
        Reset the controller to its initial state.

        Restores the robot base pose to the default position and orientation
        stored during initialization.
        """
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            self._defalut_position, self._default_orientation
        )
