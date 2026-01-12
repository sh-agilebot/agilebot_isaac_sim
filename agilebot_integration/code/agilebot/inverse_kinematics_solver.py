
import os
from typing import Optional

from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation.kinematics import InverseKinematicsSolver as BaseInverseKinematicsSolver


class InverseKinematicsSolver(BaseInverseKinematicsSolver):
    """
    The implementation of the inverse kinematics solver for the Agilebot robot.

    Args:
        name (str): The name of the inverse kinematics solver.
        robot_prim_path (str): The path to the robot prim.
        robot_type (str): The type of the robot. Optional: gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a.
        robot_urdf_path (Optional[str], optional): The path of URDF file. Defaults to None.
        robot_description_yaml_path (Optional[str], optional): The path of robot description YAML file. Defaults to None.
        end_effector_frame_name (Optional[str], optional): The name of the end effector frame. Defaults to None.
        attach_gripper (bool, optional): Whether to attach gripper. Defaults to False.
    """

    SUPPORTED_ROBOT_TYPES = {"gbt-c5a", "gbt-c7a", "gbt-c12a", "gbt-c16a"}
    DEFAULT_END_EFFECTOR_FRAME = "link6"
    GRIPPER_END_EFFECTOR_FRAME = "ee_link_robotiq_arg2f_base_link"

    def __init__(
        self,
        name: str,
        robot_prim_path: str,
        robot_type: str,
        robot_urdf_path: Optional[str] = None,
        robot_description_yaml_path: Optional[str] = None,
        end_effector_frame_name: Optional[str] = None,
        attach_gripper: bool = False,
    ) -> None:
        
        if robot_type.lower() not in self.SUPPORTED_ROBOT_TYPES:
            raise ValueError(
                f"Unsupported robot type: {robot_type}. "
                f"Supported types: {', '.join(sorted(self.SUPPORTED_ROBOT_TYPES))}"
            )
        
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        robot_type_formatted = robot_type.lower()
        if robot_urdf_path is None:
            if attach_gripper:
                robot_urdf_path = os.path.join(
                    mg_extension_path, f"motion_policy_configs/Agilebot/{robot_type_formatted}/{robot_type_formatted}_gripper.urdf"
                )
            else:
                robot_urdf_path = os.path.join(
                    mg_extension_path, f"motion_policy_configs/Agilebot/{robot_type_formatted}/{robot_type_formatted}.urdf"
                )
        if robot_description_yaml_path is None:
            if attach_gripper:
                robot_description_yaml_path = os.path.join(
                    mg_extension_path,
                    f"motion_policy_configs/Agilebot/{robot_type_formatted}/rmpflow_gripper/{robot_type_formatted}_gripper_robot_description.yaml",
                )
            else:
                robot_description_yaml_path = os.path.join(
                    mg_extension_path, f"motion_policy_configs/Agilebot/{robot_type_formatted}/rmpflow/{robot_type_formatted}_robot_description.yaml"
                )
        if end_effector_frame_name is None:
            if attach_gripper:
                end_effector_frame_name = self.GRIPPER_END_EFFECTOR_FRAME
            else:
                end_effector_frame_name = self.DEFAULT_END_EFFECTOR_FRAME
        
        if not os.path.exists(robot_urdf_path):
            raise FileNotFoundError(f"URDF file not found: {robot_urdf_path}")
        if not os.path.exists(robot_description_yaml_path):
            raise FileNotFoundError(f"Robot description YAML not found: {robot_description_yaml_path}")
        
        BaseInverseKinematicsSolver.__init__(
            self,
            name=name,
            robot_urdf_path=robot_urdf_path,
            robot_description_yaml_path=robot_description_yaml_path,
            robot_prim_path=robot_prim_path,
            end_effector_frame_name=end_effector_frame_name,
        )
