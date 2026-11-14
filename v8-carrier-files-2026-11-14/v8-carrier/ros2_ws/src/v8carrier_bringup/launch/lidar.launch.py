"""RPLIDAR C1: sllidar_ros2's sllidar_node with the C1 settings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("v8carrier_bringup"), "config", "sllidar_c1.yaml")
    serial_port = LaunchConfiguration("serial_port")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[config, {"serial_port": serial_port}],
        ),
    ])
