from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('gbt_gripper_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'gbt-c7a.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    print(urdf_path)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_path],
        parameters=[{'use_sim_time': True}]
    )

    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node
    ])
