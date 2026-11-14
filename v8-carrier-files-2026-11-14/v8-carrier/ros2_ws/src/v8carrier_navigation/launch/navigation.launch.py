"""Navigation on a saved map: Nav2's bringup (map_server + AMCL + navigation)
with the V8 carrier parameters.

  ros2 launch v8carrier_navigation navigation.launch.py map:=/home/pi/maps/home.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("v8carrier_navigation")
    nav2_params = os.path.join(share, "config", "nav2_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("map", description="map yaml saved by map_saver"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": nav2_params,
                "autostart": "true",
            }.items(),
        ),
    ])
