"""Mission node: start the vacuum, cover the map, stop the vacuum, return to
the start. State machine in mission_fsm.py.

Services   /mission/start, /mission/stop, /mission/go_home (std_srvs/Trigger)
Subscribes /mission/command (std_msgs/String: start|stop|go_home; battery_rules sends go_home)
           /mission/schedule (std_msgs/String "HH:MM" local, "" clears)
           /vacuum/state (JSON from serial_bridge), /coverage/status (JSON from coverage)
Publishes  /mission/state (std_msgs/String JSON), /vacuum/set_mode (UInt8), /brush/set (UInt8)
Calls      /coverage/start, /coverage/cancel; NavigateToPose for the return home.
"""

from __future__ import annotations

import datetime as dt
import json
import math
import time
from typing import Optional

from .mission_fsm import MissionConfig, MissionFsm

try:  # pragma: no cover
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String, UInt8
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object

VACUUM_MODE_IDS = {"off": 0, "soft": 1, "hard": 2}


class MissionNode(Node):  # pragma: no cover - needs ROS
    def __init__(self) -> None:
        super().__init__("mission")
        self.declare_parameter("vacuum_mode", "soft")
        self.declare_parameter("brush_duty", 80)
        self.declare_parameter("vacuum_start_timeout_s", 20.0)
        self.declare_parameter("vacuum_stop_timeout_s", 20.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("schedule", "")
        p = self.get_parameter
        self.fsm = MissionFsm(MissionConfig(
            vacuum_mode=str(p("vacuum_mode").value), brush_duty=int(p("brush_duty").value),
            vacuum_start_timeout_s=float(p("vacuum_start_timeout_s").value),
            vacuum_stop_timeout_s=float(p("vacuum_stop_timeout_s").value)))
        self.fsm.set_schedule(str(p("schedule").value))
        self.global_frame = p("global_frame").value
        self.base_frame = p("base_frame").value
        self.home: Optional[tuple] = None
        self.nav_handle = None

        self.pub_state = self.create_publisher(String, "mission/state", 5)
        self.pub_vacuum = self.create_publisher(UInt8, "vacuum/set_mode", 5)
        self.pub_brush = self.create_publisher(UInt8, "brush/set", 5)
        self.create_subscription(String, "mission/command", self.on_command, 5)
        self.create_subscription(String, "mission/schedule", self.on_schedule, 5)
        self.create_subscription(String, "vacuum/state", self.on_vacuum, 5)
        self.create_subscription(String, "coverage/status", self.on_coverage, 5)
        for name in ("start", "stop", "go_home"):
            self.create_service(Trigger, f"mission/{name}", lambda rq, rs, n=name: self.on_service(n, rs))
        self.cov_start = self.create_client(Trigger, "coverage/start")
        self.cov_cancel = self.create_client(Trigger, "coverage/cancel")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_coverage_status = None
        self.create_timer(1.0, self.on_timer)

    # ---- Inputs ---------------------------------------------------------------
    def on_service(self, name: str, response):
        before = self.fsm.state
        self.run(self.fsm.handle(name, time.monotonic()))
        response.success = self.fsm.state != before or name == "stop"
        response.message = f"{self.fsm.state}: {self.fsm.detail}"
        self.publish_state()
        return response

    def on_command(self, msg) -> None:
        self.get_logger().info(f"command: {msg.data}")
        self.run(self.fsm.handle(msg.data.strip(), time.monotonic()))
        self.publish_state()

    def on_schedule(self, msg) -> None:
        if self.fsm.set_schedule(msg.data):
            self.get_logger().info(f"schedule: {self.fsm.schedule or 'none'}")
        else:
            self.get_logger().warning(f"bad schedule '{msg.data}', expected HH:MM")
        self.publish_state()

    def on_vacuum(self, msg) -> None:
        try:
            v = json.loads(msg.data)
        except ValueError:
            return
        self.run(self.fsm.vacuum(v.get("mode", "unknown"), v.get("status", "idle"), time.monotonic()))

    def on_coverage(self, msg) -> None:
        try:
            status = json.loads(msg.data).get("status")
        except ValueError:
            return
        if status != self.last_coverage_status:
            self.last_coverage_status = status
            self.run(self.fsm.coverage(status, time.monotonic()))

    def on_timer(self) -> None:
        self.run(self.fsm.tick(time.monotonic(), dt.datetime.now()))
        self.publish_state()

    # ---- Actions --------------------------------------------------------------
    def run(self, actions) -> None:
        for a in actions:
            kind = a[0]
            if kind == "record_home":
                self.home = self.robot_pose()
                self.fsm.home_known = self.home is not None
                self.get_logger().info(f"home: {self.home}")
            elif kind == "vacuum":
                self.pub_vacuum.publish(UInt8(data=VACUUM_MODE_IDS[a[1]]))
            elif kind == "brush":
                self.pub_brush.publish(UInt8(data=int(a[1])))
            elif kind == "coverage":
                client = self.cov_start if a[1] == "start" else self.cov_cancel
                if client.wait_for_service(timeout_sec=2.0):
                    client.call_async(Trigger.Request()).add_done_callback(
                        lambda f, what=a[1]: self.get_logger().info(f"coverage {what}: {f.result().message}"))
                else:
                    self.get_logger().error("coverage node not available")
                    self.run(self.fsm.coverage("failed", time.monotonic()))
            elif kind == "nav_home":
                self.go_home()
            elif kind == "nav_cancel":
                if self.nav_handle is not None:
                    self.nav_handle.cancel_goal_async()
        if actions:
            self.publish_state()

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=1.0))
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"no {self.global_frame}->{self.base_frame} transform: {e}")
            return None

    def go_home(self) -> None:
        if self.home is None:
            self.run(self.fsm.nav("failed", time.monotonic()))
            return
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose not available")
            self.run(self.fsm.nav("failed", time.monotonic()))
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.home
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav.send_goal_async(goal).add_done_callback(self.on_nav_goal)

    def on_nav_goal(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.run(self.fsm.nav("failed", time.monotonic()))
            return
        self.nav_handle = handle
        handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future) -> None:
        status = future.result().status
        self.nav_handle = None
        self.run(self.fsm.nav({4: "succeeded", 5: "canceled"}.get(status, "failed"), time.monotonic()))

    def publish_state(self) -> None:
        self.pub_state.publish(String(data=json.dumps(self.fsm.status())))


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise SystemExit("rclpy is not available")
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
