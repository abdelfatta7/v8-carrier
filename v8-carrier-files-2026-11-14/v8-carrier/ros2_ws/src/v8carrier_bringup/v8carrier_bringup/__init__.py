"""v8carrier_bringup: serial bridge, sensor filters, battery rules, web UI.

Pure-logic modules (importable without ROS): protocol, odometry, sector_blank,
battery_rules, recovery. Nodes import rclpy inside main() only.
"""
