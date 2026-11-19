"""Kinect v2 driver.

    ros2 launch kinect_v2_ros2 kinect_v2.launch.py [simulate:=true] [namespace:=kinect_v2]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("kinect_v2_ros2"), "config", "kinect_v2.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("simulate", default_value="false"),
            DeclareLaunchArgument("namespace", default_value="kinect_v2"),
            DeclareLaunchArgument("params", default_value=params),
            Node(
                package="kinect_v2_ros2",
                executable="kinect_v2_node",
                name="driver",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params"),
                    {"simulate": LaunchConfiguration("simulate")},
                ],
                respawn=True,
                respawn_delay=3.0,
            ),
        ]
    )
