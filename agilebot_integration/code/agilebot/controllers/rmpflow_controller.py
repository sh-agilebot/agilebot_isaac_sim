# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from typing import Optional

import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import get_extension_path_from_name


class RMPFlowController(mg.MotionPolicyController):
    """RMPFlow Motion Policy Controller for Agilebot robotic arm motion generation.

    This controller is based on RMPFlow (Recursive Management of Policies for Flow) algorithm
    and supports both gripper-attached and gripper-free configurations.

    Args:
        name (str): Unique identifier name for the controller.
        robot_articulation (SingleArticulation): Robot articulation instance.
        physics_dt (float, optional): Physics simulation time step in seconds. Defaults to 1.0/60.0.
        attach_gripper (bool, optional): Whether to attach a gripper. Defaults to True.
        robot_type (Optional[str], optional): Robot type identifier. Defaults to None.
        end_effector_frame_name (str, optional): End effector frame name. Defaults to "ee_link_robotiq_arg2f_base_link".
    """

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        attach_gripper: bool = True,
        robot_type: str= None,
        end_effector_frame_name: str = "ee_link_robotiq_arg2f_base_link",
    ) -> None:

        if robot_type is None:
            raise ValueError(
                "robot_type must be provided. "
                "Supported robot_type values: 'gbt-c5a,gbt-c7a,gbt-c12a,gbt-c16a' (or customize for your robot)."
            )

        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        robot_type_formatted = robot_type.lower().replace('_', '-')

        config_dir = os.path.join(
            mg_extension_path,
            "motion_policy_configs",
            "Agilebot",
            robot_type_formatted
        )

        if attach_gripper:
            subdir = "rmpflow_gripper"
            urdf_filename = f"{robot_type_formatted}_gripper.urdf"
            robot_desc_filename = f"{robot_type_formatted}_gripper_robot_description.yaml"
            rmpflow_config_filename = f"{robot_type_formatted}_gripper_rmpflow_config.yaml"
            ee_frame = end_effector_frame_name
        else:
            subdir = "rmpflow"
            urdf_filename = f"{robot_type_formatted}.urdf"
            robot_desc_filename = f"{robot_type_formatted}_robot_description.yaml"
            rmpflow_config_filename = f"{robot_type_formatted}_rmpflow_config.yaml"
            ee_frame = "link6"

        robot_desc_path = os.path.join(config_dir, subdir, robot_desc_filename)
        urdf_path = os.path.join(config_dir, urdf_filename)
        rmpflow_config_path = os.path.join(config_dir, subdir, rmpflow_config_filename)

        print(f"RMPFlow config path: {config_dir}")
        print(f"Robot description file: {robot_desc_path}")
        print(f"URDF file: {urdf_path}")
        print(f"RMPFlow config file: {rmpflow_config_path}")

        self.rmp_flow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=robot_desc_path,
            urdf_path=urdf_path,
            rmpflow_config_path=rmpflow_config_path,
            end_effector_frame_name=ee_frame,
            maximum_substep_size=0.00334,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation, self.rmp_flow, physics_dt
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
