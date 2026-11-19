"""Kinect v2 driver.

    ros2 launch kinect_v2_ros2 kinect_v2.launch.py [simulate:=true] [namespace:=kinect_v2]
    Second Kinect: namespace:=kinect_flur serial:=<serial> (frames kinect_flur_...)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("kinect_v2_ros2"), "config", "kinect_v2.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("simulate", default_value="false"),
            DeclareLaunchArgument("namespace", default_value="kinect_v2"),
            DeclareLaunchArgument("params", default_value=params),
            DeclareLaunchArgument("serial", default_value=""),
            DeclareLaunchArgument(
                "frame_prefix", default_value=LaunchConfiguration("namespace")
            ),
            Node(
                package="kinect_v2_ros2",
                executable="kinect_v2_node",
                name="driver",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params"),
                    {
                        "simulate": LaunchConfiguration("simulate"),
                        # serial numbers consist of digits - keep them a string
                        "serial": ParameterValue(
                            LaunchConfiguration("serial"), value_type=str
                        ),
                        "frame_prefix": LaunchConfiguration("frame_prefix"),
                    },
                ],
                respawn=True,
                respawn_delay=3.0,
            ),
        ]
    )
