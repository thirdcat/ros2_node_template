import os
from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = os.path.join(
        get_package_share_directory("ros2_node_template"))
    rviz_config_path = os.path.join(
        pkg_path, "config", "rviz.rviz"
    )

    config = os.path.join(
        pkg_path, "config", "params.yaml")

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package='ros2_node_template',
            executable='ros2_node_template',
            output='screen',
            emulate_tty=True,
            parameters=[
                config
            ]
        )
    ])
