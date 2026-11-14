"""URDF generator for the V8 carrier, driven by cad/params.yaml.

Frames (all metres, ROS conventions: x forward, y left, z up):
  base_footprint  on the floor, under the axle midpoint (the odometry frame's child)
  base_link       axle midpoint, wheel-radius above the floor; carries the body box
  wheel_left/right  continuous joints at +-track/2 on the axle
  caster_link     ball caster at the rear
  laser           RPLIDAR C1 scan plane on the nose tower
  imu_link        IMU in the electronics (right) pod

Pure Python: `generate_urdf(params_dict)` returns the XML string; `load_params`
reads the YAML. The CLI (`generate_urdf` entry point / `python -m`) writes
urdf/v8carrier.urdf. Tests check the frames against the numbers in params.yaml.
"""

from __future__ import annotations

import math
import os
from typing import Dict
from xml.sax.saxutils import quoteattr

import yaml

# Things cad/params.yaml does not decide yet (the CAD owns them later); kept
# here in one place so they are easy to update.
BODY_GROUND_CLEARANCE_MM = 20.0      # body underside above the floor
LIDAR_BASE_ABOVE_FLOOR_MM = None     # None = on top of the body (body.height)
C1_SCAN_PLANE_ABOVE_BASE_MM = 27.0   # RPLIDAR C1 optical centre above its base (approx., 42 mm tall)
IMU_ABOVE_FLOOR_MM = 80.0            # IMU board height in the right pod
WHEEL_MASS_KG_DEFAULT = 0.08
CASTER_MASS_KG = 0.03


def load_params(path: str) -> Dict:
    with open(path) as f:
        return yaml.safe_load(f)


def find_default_params_path() -> str:
    """cad/params.yaml relative to this source file (repo checkout), else cwd."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, "..", "..", "..", "..", "cad", "params.yaml")),
        os.path.join(os.getcwd(), "cad", "params.yaml"),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    raise FileNotFoundError("cad/params.yaml not found; pass the path explicitly")


def mm(v: float) -> float:
    return float(v) / 1000.0


def frames_from_params(p: Dict) -> Dict:
    """Every number the URDF needs, in metres, derived from params.yaml."""
    body = p["body"]
    wheels = p["wheels"]
    caster = p["caster"]
    lidar = p["lidar"]
    pods = p["pods"]
    slot = p["slot"]
    masses = p.get("masses", {})

    wheel_r = mm(wheels["diameter"]) / 2.0
    axle_from_front = mm(wheels["axle_from_front"])
    length = mm(body["length"])
    width = mm(body["width"])
    height = mm(body["height"])
    clearance = mm(BODY_GROUND_CLEARANCE_MM)

    # x positions relative to the axle (base_link origin).
    front_x = axle_from_front
    rear_x = axle_from_front - length
    body_centre_x = 0.5 * (front_x + rear_x)
    body_centre_z_floor = clearance + height / 2.0

    lidar_base = height if LIDAR_BASE_ABOVE_FLOOR_MM is None else mm(LIDAR_BASE_ABOVE_FLOOR_MM)
    laser_z_floor = lidar_base + mm(C1_SCAN_PLANE_ABOVE_BASE_MM)
    laser_x = front_x - mm(lidar["position_from_front"])

    caster_r = mm(caster["ball_diameter"]) / 2.0
    caster_x = rear_x + mm(caster["position_from_rear"])

    imu_y = -(mm(slot["width"]) / 2.0 + mm(pods["width"]) / 2.0)  # right pod = electronics

    chassis_mass = (masses.get("printed_chassis_total", 1.6) + masses.get("vacuum", 1.6)
                    + masses.get("battery_4s", 0.45) + masses.get("pi5_with_cooler", 0.08)
                    + 2 * masses.get("motor_drive_each", 0.2) + masses.get("lidar", 0.11)
                    + masses.get("brush_roll", 0.15) + masses.get("brush_motor", 0.1))

    # Planning footprint: the body, widened to the wheels when they run
    # outside the pods (track_width + wheel width > body width).
    half_y = max(width / 2.0, mm(wheels["track_width"]) / 2.0 + mm(wheels["width"]) / 2.0)

    return {
        "wheel_radius": wheel_r,
        "wheel_width": mm(wheels["width"]),
        "track_width": mm(wheels["track_width"]),
        "body_size": (length, width, height),
        "body_centre_in_base_link": (body_centre_x, 0.0, body_centre_z_floor - wheel_r),
        "base_link_z": wheel_r,
        "laser_in_base_link": (laser_x, 0.0, laser_z_floor - wheel_r),
        "laser_size": (mm(lidar["diameter"]), mm(lidar["height"])),
        "caster_in_base_link": (caster_x, 0.0, caster_r - wheel_r),
        "caster_radius": caster_r,
        "imu_in_base_link": (0.0, imu_y, mm(IMU_ABOVE_FLOOR_MM) - wheel_r),
        "chassis_mass": chassis_mass,
        "wheel_mass": masses.get("wheel_each", WHEEL_MASS_KG_DEFAULT),
        "footprint_x": (rear_x, front_x),
        "footprint_y": (-half_y, half_y),
        "robot_radius": math.hypot(max(abs(rear_x), abs(front_x)), half_y),
    }


def _box_inertia(m, x, y, z):
    return (m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z), m / 12.0 * (x * x + y * y))


def _cylinder_inertia(m, r, h):
    return (m / 12.0 * (3 * r * r + h * h), m / 12.0 * (3 * r * r + h * h), m / 2.0 * r * r)


def _fmt(*vals) -> str:
    return " ".join(f"{v:.4f}" for v in vals)


def generate_urdf(params: Dict, robot_name: str = "v8carrier") -> str:
    f = frames_from_params(params)
    L, W, H = f["body_size"]
    bx, by, bz = f["body_centre_in_base_link"]
    wr, ww = f["wheel_radius"], f["wheel_width"]
    half_track = f["track_width"] / 2.0
    lx, ly, lz = f["laser_in_base_link"]
    ld, lh = f["laser_size"]
    cx, cy, cz = f["caster_in_base_link"]
    cr = f["caster_radius"]
    ix, iy, iz = f["imu_in_base_link"]
    m_body = f["chassis_mass"]
    m_wheel = f["wheel_mass"]
    ibx, iby, ibz = _box_inertia(m_body, L, W, H)
    iwx, iwy, iwz = _cylinder_inertia(m_wheel, wr, ww)
    icx = 2.0 / 5.0 * CASTER_MASS_KG * cr * cr

    def wheel(name: str, y: float) -> str:
        return f"""
  <link name="wheel_{name}">
    <visual>
      <origin xyz="0 0 0" rpy="{_fmt(math.pi / 2, 0, 0)}"/>
      <geometry><cylinder radius="{wr:.4f}" length="{ww:.4f}"/></geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="{_fmt(math.pi / 2, 0, 0)}"/>
      <geometry><cylinder radius="{wr:.4f}" length="{ww:.4f}"/></geometry>
    </collision>
    <inertial>
      <mass value="{m_wheel:.3f}"/>
      <inertia ixx="{iwx:.6f}" ixy="0" ixz="0" iyy="{iwz:.6f}" iyz="0" izz="{iwy:.6f}"/>
    </inertial>
  </link>
  <joint name="wheel_{name}_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_{name}"/>
    <origin xyz="{_fmt(0.0, y, 0.0)}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>"""

    urdf = f"""<?xml version="1.0"?>
<!-- GENERATED by v8carrier_description/urdf_generator.py from cad/params.yaml.
     Do not edit by hand: run `ros2 run v8carrier_description generate_urdf`
     (or `python3 -m v8carrier_description.urdf_generator`) after changing the
     parameters. Units: metres, kilograms. -->
<robot name={quoteattr(robot_name)}>
  <material name="shell"><color rgba="0.85 0.85 0.82 1"/></material>
  <material name="dark"><color rgba="0.15 0.15 0.15 1"/></material>
  <material name="sensor"><color rgba="0.2 0.3 0.6 1"/></material>

  <!-- On the floor under the axle midpoint: the frame odometry and Nav2 use. -->
  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="{_fmt(0.0, 0.0, f['base_link_z'])}" rpy="0 0 0"/>
  </joint>

  <!-- Axle midpoint, wheel-radius above the floor. The body box is
       {L * 1000:.0f} x {W * 1000:.0f} x {H * 1000:.0f} mm, its front {f['footprint_x'][1] * 1000:.0f} mm ahead of the axle. -->
  <link name="base_link">
    <visual>
      <origin xyz="{_fmt(bx, by, bz)}" rpy="0 0 0"/>
      <geometry><box size="{_fmt(L, W, H)}"/></geometry>
      <material name="shell"/>
    </visual>
    <collision>
      <origin xyz="{_fmt(bx, by, bz)}" rpy="0 0 0"/>
      <geometry><box size="{_fmt(L, W, H)}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="{_fmt(bx, by, bz)}" rpy="0 0 0"/>
      <mass value="{m_body:.3f}"/>
      <inertia ixx="{ibx:.5f}" ixy="0" ixz="0" iyy="{iby:.5f}" iyz="0" izz="{ibz:.5f}"/>
    </inertial>
  </link>
{wheel("left", half_track)}
{wheel("right", -half_track)}

  <link name="caster_link">
    <visual>
      <geometry><sphere radius="{cr:.4f}"/></geometry>
      <material name="dark"/>
    </visual>
    <collision>
      <geometry><sphere radius="{cr:.4f}"/></geometry>
    </collision>
    <inertial>
      <mass value="{CASTER_MASS_KG:.3f}"/>
      <inertia ixx="{icx:.7f}" ixy="0" ixz="0" iyy="{icx:.7f}" iyz="0" izz="{icx:.7f}"/>
    </inertial>
  </link>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_link"/>
    <origin xyz="{_fmt(cx, cy, cz)}" rpy="0 0 0"/>
  </joint>

  <!-- RPLIDAR C1 on the nose tower, {params['lidar']['position_from_front']} mm behind the front.
       The scan plane is {lz + f['base_link_z']:.3f} m above the floor (approximate until the tower is built). -->
  <link name="laser">
    <visual>
      <origin xyz="{_fmt(0.0, 0.0, -mm(C1_SCAN_PLANE_ABOVE_BASE_MM) + lh / 2.0)}" rpy="0 0 0"/>
      <geometry><cylinder radius="{ld / 2.0:.4f}" length="{lh:.4f}"/></geometry>
      <material name="sensor"/>
    </visual>
    <inertial>
      <mass value="0.110"/>
      <inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00004"/>
    </inertial>
  </link>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="{_fmt(lx, ly, lz)}" rpy="0 0 0"/>
  </joint>

  <!-- IMU in the electronics pod (right side). Mount the board with its x
       axis forward and z up, or fix the orientation here. -->
  <link name="imu_link">
    <inertial>
      <mass value="0.005"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="{_fmt(ix, iy, iz)}" rpy="0 0 0"/>
  </joint>
</robot>
"""
    return urdf


def nav2_footprint(params: Dict) -> list:
    """Footprint polygon (base_footprint frame) for Nav2, as [[x, y], ...]."""
    f = frames_from_params(params)
    (x0, x1), (y0, y1) = f["footprint_x"], f["footprint_y"]
    return [[round(x1, 3), round(y1, 3)], [round(x1, 3), round(y0, 3)],
            [round(x0, 3), round(y0, 3)], [round(x0, 3), round(y1, 3)]]


def generate_geometry_yaml(params: Dict) -> str:
    """ROS parameter file with the geometry the bringup/navigation nodes need,
    so cad/params.yaml stays the single source of truth. Loaded after the
    hand-written config files in the launch files, so it overrides them."""
    f = frames_from_params(params)
    fp = nav2_footprint(params)
    fp_str = "[" + ", ".join(f"[{x}, {y}]" for x, y in fp) + "]"
    return (
        "# GENERATED by v8carrier_description/urdf_generator.py from cad/params.yaml. Do not edit.\n"
        "# Geometry parameters for serial_bridge (odometry), coverage (row spacing uses\n"
        "# the brush, robot_radius the inflation) and as the reference for the Nav2\n"
        "# footprint in v8carrier_navigation/config/nav2_params.yaml (checked by a test).\n"
        "/**:\n"
        "  ros__parameters:\n"
        f"    wheel_diameter: {f['wheel_radius'] * 2:.4f}\n"
        f"    track_width: {f['track_width']:.4f}\n"
        f"    robot_radius: {f['robot_radius']:.3f}\n"
        f"    brush_width: {mm(params['brush_roll']['length']):.3f}\n"
        f"    footprint: \"{fp_str}\"\n"
    )


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Write the V8 carrier URDF and geometry.yaml from cad/params.yaml")
    ap.add_argument("--params", default=None, help="path to cad/params.yaml (default: find it)")
    ap.add_argument("--out", default=None,
                    help="URDF output file (default: urdf/v8carrier.urdf next to this package's source)")
    ap.add_argument("--geometry-out", default=None,
                    help="geometry.yaml output file (default: config/geometry.yaml next to this package's source)")
    args = ap.parse_args(argv)
    params_path = args.params or find_default_params_path()
    pkg_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    out = args.out or os.path.join(pkg_dir, "urdf", "v8carrier.urdf")
    geo_out = args.geometry_out or os.path.join(pkg_dir, "config", "geometry.yaml")
    params = load_params(params_path)
    for path, text in ((out, generate_urdf(params)), (geo_out, generate_geometry_yaml(params))):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)
        print(f"wrote {path} from {params_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
