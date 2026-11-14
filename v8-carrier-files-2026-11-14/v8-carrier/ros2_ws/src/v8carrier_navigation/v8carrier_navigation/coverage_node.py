"""Coverage node: plans boustrophedon rows over the map and drives them with
Nav2's FollowWaypoints action. Planning logic in coverage_planner.py.

Map source: the `map_yaml` parameter (a map_saver file), else the latched
/map topic (map_server). The robot's pose comes from TF map->base_footprint.

Services  /coverage/start, /coverage/cancel (std_srvs/Trigger)
Publishes /coverage/status (std_msgs/String JSON: status, waypoints, current, length_m)
          /coverage/path (nav_msgs/Path, for RViz)
"""

from __future__ import annotations

import json
import math
import threading
from typing import List, Optional

import numpy as np

from .coverage_planner import CoverageConfig, Waypoint, path_length, plan_coverage
from .map_io import load_map

try:  # pragma: no cover
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowWaypoints
    from nav_msgs.msg import OccupancyGrid, Path
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object


def pose_stamped(frame: str, stamp, x: float, y: float, yaw: float):  # pragma: no cover
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = stamp
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class CoverageNode(Node):  # pragma: no cover - needs ROS
    def __init__(self) -> None:
        super().__init__("coverage")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("row_spacing", 0.22)
        self.declare_parameter("brush_width", 0.24)
        self.declare_parameter("overlap", 0.02)
        self.declare_parameter("robot_radius", 0.291)
        self.declare_parameter("margin", 0.03)
        self.declare_parameter("min_segment", 0.30)
        self.declare_parameter("row_axis", "auto")
        self.declare_parameter("use_row_spacing", False)  # False: spacing = brush_width - overlap

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map: Optional[tuple] = None  # (grid, resolution, origin)
        self.lock = threading.Lock()
        self.status = "idle"
        self.waypoints: List[Waypoint] = []
        self.current = 0
        self.goal_handle = None

        map_yaml = self.get_parameter("map_yaml").value
        if map_yaml:
            m = load_map(map_yaml)
            self.map = (m.grid, m.resolution, m.origin)
            self.get_logger().info(f"map from {map_yaml}: {m.grid.shape[1]}x{m.grid.shape[0]} @ {m.resolution} m")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.pub_status = self.create_publisher(String, "coverage/status", 5)
        self.pub_path = self.create_publisher(Path, "coverage/path", latched)
        self.create_service(Trigger, "coverage/start", self.on_start)
        self.create_service(Trigger, "coverage/cancel", self.on_cancel)
        self.create_timer(1.0, self.publish_status)

    # ---- Inputs ---------------------------------------------------------------
    def on_map(self, msg) -> None:
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        with self.lock:
            self.map = (grid, msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y))

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=1.0))
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:  # noqa: BLE001 - any TF failure: plan without a start
            self.get_logger().warning(f"no {self.global_frame}->{self.base_frame} transform: {e}")
            return None

    def config(self) -> CoverageConfig:
        p = self.get_parameter
        spacing = float(p("row_spacing").value) if bool(p("use_row_spacing").value) else \
            float(p("brush_width").value) - float(p("overlap").value)
        return CoverageConfig(row_spacing=spacing, robot_radius=float(p("robot_radius").value),
                              margin=float(p("margin").value), min_segment=float(p("min_segment").value),
                              row_axis=str(p("row_axis").value))

    # ---- Services -------------------------------------------------------------
    def on_start(self, request, response):
        if self.status == "running":
            response.success = False
            response.message = "already running"
            return response
        with self.lock:
            m = self.map
        if m is None:
            response.success = False
            response.message = "no map yet"
            return response
        self.status = "planning"
        grid, res, origin = m
        wps = plan_coverage(grid, res, origin, self.config(), self.robot_xy())
        if not wps:
            self.status = "failed"
            response.success = False
            response.message = "no free rows in the map"
            return response
        self.waypoints = wps
        self.current = 0
        self.publish_path()
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.status = "failed"
            response.success = False
            response.message = "follow_waypoints action server not available"
            return response
        goal = FollowWaypoints.Goal()
        stamp = self.get_clock().now().to_msg()
        goal.poses = [pose_stamped(self.global_frame, stamp, x, y, yaw) for x, y, yaw in wps]
        self.status = "running"
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        response.success = True
        response.message = f"{len(wps)} waypoints, {path_length(wps):.1f} m"
        self.get_logger().info("coverage started: " + response.message)
        return response

    def on_cancel(self, request, response):
        if self.goal_handle is not None and self.status == "running":
            self.goal_handle.cancel_goal_async()
            self.status = "canceled"
            response.success = True
            response.message = "cancel requested"
        else:
            response.success = False
            response.message = "not running"
        return response

    # ---- Action callbacks -----------------------------------------------------
    def on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.status = "failed"
            self.get_logger().error("follow_waypoints rejected the goal")
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg) -> None:
        self.current = int(msg.feedback.current_waypoint)

    def on_result(self, future) -> None:
        result = future.result()
        if self.status == "canceled":
            pass
        elif result.status == 4:  # GoalStatus.STATUS_SUCCEEDED
            missed = list(result.result.missed_waypoints)
            self.status = "succeeded"
            if missed:
                self.get_logger().warning(f"missed waypoints: {missed}")
        else:
            self.status = "failed"
        self.goal_handle = None
        self.publish_status()

    # ---- Outputs --------------------------------------------------------------
    def publish_status(self) -> None:
        self.pub_status.publish(String(data=json.dumps({
            "status": self.status, "waypoints": len(self.waypoints), "current": self.current,
            "length_m": round(path_length(self.waypoints), 1) if self.waypoints else 0.0})))

    def publish_path(self) -> None:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [pose_stamped(self.global_frame, path.header.stamp, x, y, yaw) for x, y, yaw in self.waypoints]
        self.pub_path.publish(path)


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise SystemExit("rclpy is not available")
    rclpy.init(args=args)
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
