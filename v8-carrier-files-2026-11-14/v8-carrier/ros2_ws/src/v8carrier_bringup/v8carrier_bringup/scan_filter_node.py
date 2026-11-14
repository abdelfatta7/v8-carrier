"""Scan filter: republishes /scan as /scan_filtered with the sector behind the
LiDAR that the vacuum body blocks set to NaN. Logic in sector_blank.py.

Parameters: blank_start_deg (130), blank_end_deg (230): counter-clockwise
sector in the laser frame, 0 = ahead. Default = the rear 100 degrees.
"""

from __future__ import annotations

import math

from .sector_blank import blank_sector

try:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object


class ScanFilter(Node):  # pragma: no cover - needs ROS
    def __init__(self) -> None:
        super().__init__("scan_filter")
        self.declare_parameter("input_topic", "scan")
        self.declare_parameter("output_topic", "scan_filtered")
        self.declare_parameter("blank_start_deg", 130.0)
        self.declare_parameter("blank_end_deg", 230.0)
        self.start = float(self.get_parameter("blank_start_deg").value)
        self.end = float(self.get_parameter("blank_end_deg").value)
        self.pub = self.create_publisher(LaserScan, self.get_parameter("output_topic").value,
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter("input_topic").value, self.on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"blanking {self.start:.0f}..{self.end:.0f} deg")

    def on_scan(self, msg) -> None:
        out = blank_sector(msg.ranges, msg.angle_min, msg.angle_increment, self.start, self.end, math.nan)
        msg.ranges = out.tolist()
        self.pub.publish(msg)


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise SystemExit("rclpy is not available")
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
