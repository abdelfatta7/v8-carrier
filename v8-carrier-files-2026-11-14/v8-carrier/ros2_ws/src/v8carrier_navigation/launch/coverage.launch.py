"""Coverage and mission nodes (run after bringup and navigation).

  ros2 launch v8carrier_navigation coverage.launch.py [map_yaml:=/home/pi/maps/home.yaml]
The map is taken from the latched /map topic unless map_yaml is given.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("v8carrier_navigation")
    config = os.path.join(share, "config", "coverage.yaml")
    geometry = os.path.join(get_package_share_directory("v8carrier_description"), "config", "geometry.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", default_value=""),
        Node(package="v8carrier_navigation", executable="coverage", name="coverage", output="screen",
             parameters=[config, geometry, {"map_yaml": LaunchConfiguration("map_yaml")}]),
        Node(package="v8carrier_navigation", executable="mission", name="mission", output="screen",
             parameters=[config]),
    ])
