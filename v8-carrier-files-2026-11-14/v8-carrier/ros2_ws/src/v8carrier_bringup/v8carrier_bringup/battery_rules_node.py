"""Battery rules node: sends the robot home when the pack runs low during a
mission or when the vacuum stops by itself. Logic in battery_rules.py.

Subscribes /battery_state, /vacuum/state, /mission/state.
Publishes  /mission/command (std_msgs/String: "go_home").
"""

from __future__ import annotations

import json
import time

from .battery_rules import BatteryRules, BatteryRulesConfig

try:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object


class BatteryRulesNode(Node):  # pragma: no cover - needs ROS
    def __init__(self) -> None:
        super().__init__("battery_rules")
        self.declare_parameter("low_voltage_v", 14.4)
        self.declare_parameter("low_hold_s", 10.0)
        self.declare_parameter("low_clear_v", 14.8)
        self.rules = BatteryRules(BatteryRulesConfig(
            low_voltage_v=float(self.get_parameter("low_voltage_v").value),
            low_hold_s=float(self.get_parameter("low_hold_s").value),
            low_clear_v=float(self.get_parameter("low_clear_v").value)))
        self.pack_v = None
        self.vacuum_mode = None
        self.mission_state = None
        self.pub = self.create_publisher(String, "mission/command", 5)
        self.create_subscription(BatteryState, "battery_state", self.on_battery, 5)
        self.create_subscription(String, "vacuum/state", self.on_vacuum, 5)
        self.create_subscription(String, "mission/state", self.on_mission, 5)
        self.create_timer(0.5, self.on_timer)

    def on_battery(self, msg) -> None:
        self.pack_v = float(msg.voltage)

    def on_vacuum(self, msg) -> None:
        try:
            self.vacuum_mode = json.loads(msg.data).get("mode")
        except ValueError:
            pass

    def on_mission(self, msg) -> None:
        try:
            self.mission_state = json.loads(msg.data).get("state")
        except ValueError:
            pass

    def on_timer(self) -> None:
        cmd = self.rules.update(time.monotonic(), self.pack_v, self.vacuum_mode, self.mission_state)
        if cmd:
            self.get_logger().warning(f"{cmd}: {self.rules.last_reason}")
            self.pub.publish(String(data=cmd))


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise SystemExit("rclpy is not available")
    rclpy.init(args=args)
    node = BatteryRulesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
