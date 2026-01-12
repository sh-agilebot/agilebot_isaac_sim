import os
from typing import Optional

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Agilebot robot. This class loads a LulaKinematicsSolver object.

    Args:
        robot_articulation (SingleArticulation): An initialized Articulation object representing this Agilebot robot.
        robot_type (str): The type of the robot. Optional: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a.        
        end_effector_frame_name (Optional[str]): The name of the Agilebot end effector. If None, an end effector link will
            be automatically selected. Defaults to None.
        attach_gripper (Optional[bool]): If True, a URDF will be loaded that includes a gripper. Defaults to True.
    """

    SUPPORTED_ROBOT_TYPES = {"gbt-c5a", "gbt-c7a", "gbt-c12a", "gbt-c16a"}
    DEFAULT_END_EFFECTOR_FRAME = "link6"
    GRIPPER_END_EFFECTOR_FRAME = "ee_link_robotiq_arg2f_base_link"

    def __init__(
        self,
        robot_articulation: SingleArticulation,
        robot_type: str,
        end_effector_frame_name: Optional[str] = None,
        attach_gripper: Optional[bool] = True,
    ) -> None:

        if robot_type.lower() not in self.SUPPORTED_ROBOT_TYPES:
            raise ValueError(
                f"Unsupported robot type: {robot_type}. "
                f"Supported types: {', '.join(sorted(self.SUPPORTED_ROBOT_TYPES))}"
            )

        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")

        robot_type_formatted = robot_type.lower()
        if attach_gripper:
            robot_urdf_path = os.path.join(
                mg_extension_path, f"motion_policy_configs/Agilebot/{robot_type_formatted}/{robot_type_formatted}_gripper.urdf"
            )
            robot_description_yaml_path = os.path.join(
                mg_extension_path,
                f"motion_policy_configs/Agilebot/{robot_type_formatted}/rmpflow_gripper/{robot_type_formatted}_gripper_robot_description.yaml",
            )
        else:
            robot_urdf_path = os.path.join(
                mg_extension_path, f"motion_policy_configs/Agilebot/{robot_type_formatted}/{robot_type_formatted}.urdf"
            )
            robot_description_yaml_path = os.path.join(
                mg_extension_path, f"motion_policy_configs/Agilebot/{robot_type_formatted}/rmpflow/{robot_type_formatted}_robot_description.yaml"
            )
        
        if not os.path.exists(robot_urdf_path):
            raise FileNotFoundError(f"URDF file not found: {robot_urdf_path}")
        if not os.path.exists(robot_description_yaml_path):
            raise FileNotFoundError(f"Robot description YAML not found: {robot_description_yaml_path}")

        self._kinematics = LulaKinematicsSolver(
            robot_description_path=robot_description_yaml_path, urdf_path=robot_urdf_path
        )

        if end_effector_frame_name is None:
            if attach_gripper:
                end_effector_frame_name = self.GRIPPER_END_EFFECTOR_FRAME
            else:
                end_effector_frame_name = self.DEFAULT_END_EFFECTOR_FRAME

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
