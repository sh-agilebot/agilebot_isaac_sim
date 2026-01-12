# Define an inverse kinematics solver
import os
from typing import Optional
from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver,LulaKinematicsSolver

class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self,robot_articulation:Articulation,end_effector_prim_name:Optional[str]=None)->None:
        # Define a kinematics solver
        self._kinematics=LulaKinematicsSolver(
            robot_description_path=os.path.join(os.path.dirname(__file__),"../rmpflow/gbt_c5a_camera_gripper_robot_description.yaml"),
            urdf_path=os.path.join(os.path.dirname(__file__),"../rmpflow/gbt-c5a.urdf"),
        )
        if end_effector_prim_name is None:
            end_effector_prim_name="robotiq_arg2f_base_link"
        ArticulationKinematicsSolver.__init__(self,robot_articulation,self._kinematics,end_effector_prim_name)