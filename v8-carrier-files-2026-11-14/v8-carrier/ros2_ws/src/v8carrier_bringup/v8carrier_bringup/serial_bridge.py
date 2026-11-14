"""Serial bridge: the ROS 2 face of the ESP32 (docs/protocol.md).

Publishes  /odom (nav_msgs/Odometry, + TF odom->base_footprint when publish_tf)
           /imu/data (sensor_msgs/Imu)
           /battery_state (sensor_msgs/BatteryState)
           /bumper (std_msgs/UInt8, bit0 left, bit1 right)
           /cliff (std_msgs/UInt16MultiArray, mV x4)
           /vacuum/state (std_msgs/String, JSON: mode, status, mic_running, mic_level)
           /estop (std_msgs/String, JSON: latched, active, latch_causes)
Subscribes /cmd_vel (geometry_msgs/Twist)
           /vacuum/set_mode (std_msgs/UInt8: 0 off, 1 soft, 2 hard)
           /brush/set (std_msgs/UInt8: duty 0..100)
           /servo/set (std_msgs/UInt8: degrees, calibration only)
Services   /estop/clear (std_srvs/Trigger)
Sends the heartbeat at 10 Hz and runs the estop recovery (recovery.py).

rclpy is imported lazily so the pure-logic modules next to this file import
without ROS; this node only runs on the robot.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

from .odometry import OdomParams, Odometry, twist_to_wheels
from .protocol import (ACK, ACK_OK, LOG, STATUS, Ack, FrameParser, Status, clear_estop, heartbeat,
                       set_brush, set_servo, set_stream, set_vacuum_mode, set_wheel_vel)
from .recovery import EstopRecovery, RecoveryConfig

try:  # pragma: no cover - only on the robot
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Quaternion, TransformStamped, Twist
    from nav_msgs.msg import Odometry as OdometryMsg
    from sensor_msgs.msg import BatteryState, Imu
    from std_msgs.msg import String, UInt8, UInt16MultiArray
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def pack_percentage(voltage: float, cells: int = 4) -> float:
    """Rough Li-ion state of charge from the resting voltage: 3.5 V = 0, 4.2 V = 1."""
    per_cell = voltage / cells
    return max(0.0, min(1.0, (per_cell - 3.5) / (4.2 - 3.5)))


class SerialBridge(Node):  # pragma: no cover - needs ROS
    def __init__(self) -> None:
        super().__init__("serial_bridge")
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("ticks_per_rev", 1320)
        self.declare_parameter("wheel_diameter", 0.080)
        self.declare_parameter("track_width", 0.240)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("slow_rate_divider", 10)  # battery/vacuum/estop at 50/10 = 5 Hz
        self.declare_parameter("auto_clear_heartbeat", True)
        self.declare_parameter("auto_backoff", True)
        self.declare_parameter("backoff_speed_mm_s", 60)
        self.declare_parameter("backoff_s", 1.0)
        self.declare_parameter("battery_cells", 4)
        self.declare_parameter("battery_capacity_ah", 5.0)

        p = self.get_parameter
        self.port = p("port").value
        self.baud = int(p("baud").value)
        self.odom_params = OdomParams(int(p("ticks_per_rev").value), float(p("wheel_diameter").value),
                                      float(p("track_width").value))
        self.odom = Odometry(self.odom_params)
        self.publish_tf = bool(p("publish_tf").value)
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.imu_frame = p("imu_frame").value
        self.cmd_timeout = float(p("cmd_timeout").value)
        self.slow_div = int(p("slow_rate_divider").value)
        self.cells = int(p("battery_cells").value)
        self.capacity_ah = float(p("battery_capacity_ah").value)
        self.recovery = EstopRecovery(RecoveryConfig(
            auto_clear_heartbeat=bool(p("auto_clear_heartbeat").value),
            auto_backoff=bool(p("auto_backoff").value),
            backoff_speed_mm_s=int(p("backoff_speed_mm_s").value),
            backoff_s=float(p("backoff_s").value)))

        self.pub_odom = self.create_publisher(OdometryMsg, "odom", 10)
        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)
        self.pub_batt = self.create_publisher(BatteryState, "battery_state", 5)
        self.pub_bumper = self.create_publisher(UInt8, "bumper", 5)
        self.pub_cliff = self.create_publisher(UInt16MultiArray, "cliff", 5)
        self.pub_vacuum = self.create_publisher(String, "vacuum/state", 5)
        self.pub_estop = self.create_publisher(String, "estop", 5)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(UInt8, "vacuum/set_mode", self.on_vacuum_mode, 5)
        self.create_subscription(UInt8, "brush/set", self.on_brush, 5)
        self.create_subscription(UInt8, "servo/set", self.on_servo, 5)
        self.create_service(Trigger, "estop/clear", self.on_clear_estop)

        self.ser = None
        self.ser_lock = threading.Lock()
        self.parser = FrameParser()
        self.hb_seq = 0
        self.last_cmd_time = 0.0
        self.last_status: Optional[Status] = None
        self.last_status_t_ms: Optional[int] = None
        self.status_count = 0
        self.last_bumper = None
        self.sent_zero = True

        self.create_timer(1.0 / float(p("heartbeat_hz").value), self.on_heartbeat_timer)
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    # ---- Serial ---------------------------------------------------------------
    def open_port(self) -> bool:
        import serial  # pyserial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f"opened {self.port} at {self.baud}")
            self.send(set_stream(True))
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f"cannot open {self.port}: {e}; retrying")
            self.ser = None
            return False

    def send(self, frame: bytes) -> None:
        with self.ser_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(frame)
            except Exception as e:  # noqa: BLE001 - any serial failure means reconnect
                self.get_logger().error(f"serial write failed: {e}")
                self.ser = None

    def read_loop(self) -> None:
        while rclpy.ok():
            if self.ser is None:
                if not self.open_port():
                    time.sleep(2.0)
                    continue
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"serial read failed: {e}")
                with self.ser_lock:
                    self.ser = None
                continue
            if not data:
                continue
            for msg_id, payload in self.parser.feed(data):
                self.handle_frame(msg_id, payload)

    # ---- Frames from the ESP32 ------------------------------------------------
    def handle_frame(self, msg_id: int, payload: bytes) -> None:
        if msg_id == STATUS:
            try:
                status = Status.unpack(payload)
            except ValueError as e:
                self.get_logger().warning(str(e))
                return
            self.on_status(status)
        elif msg_id == ACK:
            ack = Ack.unpack(payload)
            if ack.result != ACK_OK:
                self.get_logger().debug(f"command 0x{ack.id:02x} answered {ack.result}")
        elif msg_id == LOG:
            self.get_logger().info("esp32: " + payload.decode("ascii", errors="replace"))

    def on_status(self, s: Status) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Odometry from ticks; dt from the ESP32 clock.
        if self.last_status_t_ms is None:
            dt = 0.02
        else:
            dt = ((s.t_ms - self.last_status_t_ms) & 0xFFFFFFFF) * 1e-3
        self.last_status_t_ms = s.t_ms
        self.odom.update(s.left_ticks, s.right_ticks, dt)
        self.publish_odom(stamp)
        self.publish_imu(stamp, s)

        self.status_count += 1
        if self.status_count % self.slow_div == 0:
            self.publish_slow(stamp, s)
        if s.bumper_bits != self.last_bumper:
            self.last_bumper = s.bumper_bits
            self.pub_bumper.publish(UInt8(data=s.bumper_bits))

        # Estop recovery.
        for action in self.recovery.update(time.monotonic(), s):
            if action[0] == "wheels":
                self.send(set_wheel_vel(action[1], action[2]))
            elif action[0] == "clear":
                self.get_logger().info(f"clearing estop (causes were {s.latch_causes})")
                self.send(clear_estop())
        self.last_status = s

    def publish_odom(self, stamp) -> None:
        o = self.odom
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = o.x
        msg.pose.pose.position.y = o.y
        qx, qy, qz, qw = yaw_to_quaternion(o.theta)
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg.twist.twist.linear.x = o.v
        msg.twist.twist.angular.z = o.w
        cov = [0.0] * 36
        cov[0] = cov[7] = 1e-3
        cov[14] = cov[21] = cov[28] = 1e6  # z, roll, pitch: not observed
        cov[35] = 1e-2
        msg.pose.covariance = cov
        tcov = [0.0] * 36
        tcov[0] = 1e-3
        tcov[7] = 1e-3
        tcov[14] = tcov[21] = tcov[28] = 1e6
        tcov[35] = 1e-2
        msg.twist.covariance = tcov
        self.pub_odom.publish(msg)
        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self.tf.sendTransform(t)

    def publish_imu(self, stamp, s: Status) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        msg.angular_velocity.x = s.gyro_mrad_s[0] * 1e-3
        msg.angular_velocity.y = s.gyro_mrad_s[1] * 1e-3
        msg.angular_velocity.z = s.gyro_mrad_s[2] * 1e-3
        msg.angular_velocity_covariance = [4e-4, 0.0, 0.0, 0.0, 4e-4, 0.0, 0.0, 0.0, 4e-4]
        msg.linear_acceleration.x = s.accel_mm_s2[0] * 1e-3
        msg.linear_acceleration.y = s.accel_mm_s2[1] * 1e-3
        msg.linear_acceleration.z = s.accel_mm_s2[2] * 1e-3
        msg.linear_acceleration_covariance = [4e-2, 0.0, 0.0, 0.0, 4e-2, 0.0, 0.0, 0.0, 4e-2]
        yaw = s.yaw_rad
        if yaw is None:
            msg.orientation_covariance[0] = -1.0  # not available (MPU-6050)
        else:
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            msg.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e-2]
        self.pub_imu.publish(msg)

    def publish_slow(self, stamp, s: Status) -> None:
        b = BatteryState()
        b.header.stamp = stamp
        b.voltage = s.pack_mv * 1e-3
        b.current = -s.pack_ma * 1e-3  # ROS: negative while discharging
        b.percentage = pack_percentage(b.voltage, self.cells)
        b.capacity = self.capacity_ah
        b.design_capacity = self.capacity_ah
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if s.pack_ma > 50 else \
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = s.pack_mv > 1000
        self.pub_batt.publish(b)

        self.pub_cliff.publish(UInt16MultiArray(data=list(s.cliff_mv)))
        self.pub_vacuum.publish(String(data=json.dumps({
            "mode": s.vacuum_mode_name, "status": s.vacuum_status_name,
            "mic_running": s.mic_running, "mic_level": s.mic_level, "servo_angle": s.servo_angle_deg,
            "brush_duty": s.brush_duty_pct})))
        self.pub_estop.publish(String(data=json.dumps({
            "latched": s.estop_latched, "active": s.active_causes, "latch_causes": s.latch_causes,
            "recovery": self.recovery.state})))

    # ---- Commands from ROS ----------------------------------------------------
    def on_cmd_vel(self, msg) -> None:
        if self.recovery.overriding_wheels:
            return
        left, right = twist_to_wheels(self.odom_params, msg.linear.x, msg.angular.z)
        self.send(set_wheel_vel(int(round(left * 1000)), int(round(right * 1000))))
        self.last_cmd_time = time.monotonic()
        self.sent_zero = msg.linear.x == 0.0 and msg.angular.z == 0.0

    def on_heartbeat_timer(self) -> None:
        self.hb_seq = (self.hb_seq + 1) & 0xFF
        self.send(heartbeat(self.hb_seq))
        # If cmd_vel went quiet, send one explicit zero so the stop is immediate
        # rather than waiting for the firmware's 500 ms timeout.
        if not self.sent_zero and time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.send(set_wheel_vel(0, 0))
            self.sent_zero = True

    def on_vacuum_mode(self, msg) -> None:
        if msg.data <= 2:
            self.send(set_vacuum_mode(int(msg.data)))

    def on_brush(self, msg) -> None:
        self.send(set_brush(int(msg.data)))

    def on_servo(self, msg) -> None:
        self.send(set_servo(int(msg.data)))

    def on_clear_estop(self, request, response):
        s = self.last_status
        if s is not None and s.estop_latched and (s.estop_flags & ~0x01) != 0:
            response.success = False
            response.message = f"cause still active: {s.active_causes}"
            return response
        self.send(clear_estop())
        response.success = True
        response.message = "clear sent"
        return response


def main(args=None) -> None:  # pragma: no cover
    if rclpy is None:
        raise SystemExit("rclpy is not available: this node runs on the robot with ROS 2 Jazzy")
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(set_wheel_vel(0, 0))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
