"""Web UI: a single page (web/index.html) served by Python's http.server with
Start / Stop / Go home / Clear estop / Schedule buttons and a status readout.

  GET  /                  the page
  GET  /api/status        JSON: mission, vacuum, battery, estop, bridge_age_s
  POST /api/start         -> /mission/start   (std_srvs/Trigger)
  POST /api/stop          -> /mission/stop
  POST /api/go_home       -> /mission/go_home
  POST /api/clear_estop   -> /estop/clear
  POST /api/schedule      {"time": "HH:MM"} or {"time": ""} -> /mission/schedule (std_msgs/String)

Parameters: port (8080), bind ("0.0.0.0"), page (path to index.html; default the
installed share file).
"""

from __future__ import annotations

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object

SERVICE_FOR_ACTION = {
    "start": "mission/start",
    "stop": "mission/stop",
    "go_home": "mission/go_home",
    "clear_estop": "estop/clear",
}


class WebUi(Node):  # pragma: no cover - needs ROS
    def __init__(self) -> None:
        super().__init__("web_ui")
        self.declare_parameter("port", 8080)
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("page", "")
        page = self.get_parameter("page").value
        if not page:
            page = os.path.join(get_package_share_directory("v8carrier_bringup"), "web", "index.html")
        with open(page, "rb") as f:
            self.page = f.read()

        self.lock = threading.Lock()
        self.mission = {}
        self.vacuum = {}
        self.battery = {}
        self.estop = {}
        self.last_bridge = None

        self.create_subscription(String, "mission/state", lambda m: self._set("mission", m), 5)
        self.create_subscription(String, "vacuum/state", lambda m: self._set("vacuum", m, bridge=True), 5)
        self.create_subscription(String, "estop", lambda m: self._set("estop", m, bridge=True), 5)
        self.create_subscription(BatteryState, "battery_state", self.on_battery, 5)
        self.pub_schedule = self.create_publisher(String, "mission/schedule", 5)
        self.clients = {a: self.create_client(Trigger, s) for a, s in SERVICE_FOR_ACTION.items()}

        port = int(self.get_parameter("port").value)
        bind = self.get_parameter("bind").value
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # quiet
                node.get_logger().debug(fmt % args)

            def _json(self, code, obj):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/api/status":
                    self._json(200, node.status())
                elif self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(node.page)))
                    self.end_headers()
                    self.wfile.write(node.page)
                else:
                    self._json(404, {"error": "not found"})

            def do_POST(self):
                length = int(self.headers.get("Content-Length") or 0)
                try:
                    body = json.loads(self.rfile.read(length) or b"{}")
                except ValueError:
                    body = {}
                action = self.path[len("/api/"):] if self.path.startswith("/api/") else ""
                if action == "schedule":
                    node.pub_schedule.publish(String(data=str(body.get("time", ""))))
                    self._json(200, {"success": True, "message": "schedule sent"})
                elif action in SERVICE_FOR_ACTION:
                    self._json(200, node.call(action))
                else:
                    self._json(404, {"success": False, "message": "unknown action"})

        self.server = ThreadingHTTPServer((bind, port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f"web UI on http://{bind}:{port}/")

    def _set(self, name, msg, bridge=False) -> None:
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            setattr(self, name, value)
            if bridge:
                self.last_bridge = time.monotonic()

    def on_battery(self, msg) -> None:
        with self.lock:
            self.battery = {"voltage": msg.voltage, "current": msg.current, "percentage": msg.percentage}
            self.last_bridge = time.monotonic()

    def status(self) -> dict:
        with self.lock:
            age = None if self.last_bridge is None else time.monotonic() - self.last_bridge
            return {"mission": self.mission, "vacuum": self.vacuum, "battery": self.battery,
                    "estop": self.estop, "bridge_age_s": age}

    def call(self, action: str) -> dict:
        client = self.clients[action]
        if not client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": f"{SERVICE_FOR_ACTION[action]} not available"}
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=5.0):
            return {"success": False, "message": "service call timed out"}
        try:
            r = future.result()
        except Exception as e:  # noqa: BLE001
            return {"success": False, "message": str(e)}
        return {"success": bool(r.success), "message": r.message}


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise SystemExit("rclpy is not available")
    rclpy.init(args=args)
    node = WebUi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
