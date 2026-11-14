"""Mapping run: slam_toolbox (online async) plus Nav2 navigation without a
static map, so the robot can be driven to goals in RViz while it maps.
Save the result with:  ros2 run nav2_map_server map_saver_cli -f ~/maps/home

  ros2 launch v8carrier_navigation slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("v8carrier_navigation")
    slam_params = os.path.join(share, "config", "slam_toolbox_online_async.yaml")
    nav2_params = os.path.join(share, "config", "nav2_params.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")),
            launch_arguments={"use_sim_time": use_sim_time, "params_file": nav2_params,
                              "autostart": "true"}.items(),
        ),
    ])
