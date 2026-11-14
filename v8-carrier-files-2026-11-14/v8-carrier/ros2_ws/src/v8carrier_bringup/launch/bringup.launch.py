"""Robot bringup: description, serial bridge, LiDAR, scan filter, EKF,
battery rules and the web UI. Mapping / navigation are launched separately
from v8carrier_navigation.

  ros2 launch v8carrier_bringup bringup.launch.py [port:=/dev/ttyAMA0] [lidar_port:=/dev/ttyUSB0] [use_ekf:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("v8carrier_bringup")
    description_share = get_package_share_directory("v8carrier_description")
    bridge_config = os.path.join(bringup_share, "config", "bridge.yaml")
    ekf_config = os.path.join(bringup_share, "config", "ekf.yaml")
    # Generated from cad/params.yaml; overrides the geometry in bridge.yaml.
    geometry_config = os.path.join(description_share, "config", "geometry.yaml")

    port = LaunchConfiguration("port")
    lidar_port = LaunchConfiguration("lidar_port")
    use_ekf = LaunchConfiguration("use_ekf")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyAMA0", description="ESP32 UART"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_ekf", default_value="true",
                              description="fuse odometry and IMU with robot_localization"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(description_share, "launch", "description.launch.py"))),

        # The bridge publishes odom->base_footprint itself only without the EKF.
        Node(package="v8carrier_bringup", executable="serial_bridge", name="serial_bridge", output="screen",
             parameters=[bridge_config, geometry_config, {"port": port, "publish_tf": False}],
             condition=IfCondition(use_ekf)),
        Node(package="v8carrier_bringup", executable="serial_bridge", name="serial_bridge", output="screen",
             parameters=[bridge_config, geometry_config, {"port": port, "publish_tf": True}],
             condition=UnlessCondition(use_ekf)),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[ekf_config], condition=IfCondition(use_ekf)),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "lidar.launch.py")),
            launch_arguments={"serial_port": lidar_port}.items()),
        Node(package="v8carrier_bringup", executable="scan_filter", name="scan_filter", output="screen",
             parameters=[bridge_config]),

        Node(package="v8carrier_bringup", executable="battery_rules", name="battery_rules", output="screen",
             parameters=[bridge_config]),
        Node(package="v8carrier_bringup", executable="web_ui", name="web_ui", output="screen",
             parameters=[bridge_config]),
    ])
