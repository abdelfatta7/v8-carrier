"""robot_state_publisher with the generated URDF.

  ros2 launch v8carrier_description description.launch.py [use_sim_time:=false]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory("v8carrier_description"), "urdf", "v8carrier.urdf")
    with open(urdf) as f:
        robot_description = f.read()
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        ),
        # The wheel joints are continuous; publish zero joint states so TF has
        # the wheel frames (nothing reads the wheel angles).
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
