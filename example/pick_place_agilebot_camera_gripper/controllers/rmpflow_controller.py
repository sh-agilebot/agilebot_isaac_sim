"""
Copyright © 2025 Agilebot Robotics Ltd. All rights reserved.
Author: Desheng.Li, September 4, 2025
Instruction:
Implement pick and place functionality following Isaac Sim official tutorial. This file is the motion planning module that implements an RMPFlowController class for motion control and collision detection based on RMPflow algorithm.
reference:
 https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup_tutorials/tutorial_pickplace_example.html

"""
import os
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import Articulation

class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name:str, robot_articulation:Articulation,physical_dt:float= 1.0/60.0)->None:
        self.rmpflow=mg.lula.motion_policies.RmpFlow(
            robot_description_path=os.path.join(os.path.dirname(__file__),"../rmpflow/gbt_c5a_camera_gripper_robot_description.yaml"),
            urdf_path=os.path.join(os.path.dirname(__file__),"../rmpflow/gbt-c5a.urdf"),
            rmpflow_config_path=os.path.join(os.path.dirname(__file__),"../rmpflow/gbt_c5a_rmpflow_common.yaml"),
            end_effector_frame_name="robotiq_arg2f_base_link",
            maximum_substep_size=0.00334,
        )
        self.articulation_rmp=mg.ArticulationMotionPolicy(
            robot_articulation=robot_articulation,
            motion_policy=self.rmpflow,
            default_physics_dt=physical_dt,
        )
        mg.MotionPolicyController.__init__(self,name,self.articulation_rmp)
        self._defalut_position,self._default_orientation=(
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            self._defalut_position,self._default_orientation
        )
        
        return
    
    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            self._defalut_position,self._default_orientation
        )