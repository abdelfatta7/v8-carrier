"""The generator must turn cad/params.yaml into a URDF whose frames match the
numbers in the file (whatever they are today), and the committed
urdf/v8carrier.urdf and config/geometry.yaml must be current."""

import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from v8carrier_description import urdf_generator as G

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", "..", "..", ".."))
PARAMS = os.path.join(REPO, "cad", "params.yaml")
COMMITTED_URDF = os.path.normpath(os.path.join(HERE, "..", "urdf", "v8carrier.urdf"))
COMMITTED_GEOMETRY = os.path.normpath(os.path.join(HERE, "..", "config", "geometry.yaml"))


@pytest.fixture(scope="module")
def params():
    return G.load_params(PARAMS)


@pytest.fixture(scope="module")
def urdf(params):
    return G.generate_urdf(params)


@pytest.fixture(scope="module")
def tree(urdf):
    return ET.fromstring(urdf)


def joint_origin(tree, name):
    j = tree.find(f"./joint[@name='{name}']")
    assert j is not None, name
    return [float(v) for v in j.find("origin").get("xyz").split()], j


def test_params_have_what_the_generator_needs(params):
    assert params["wheels"]["diameter"] == 80  # decided 2026-11-10
    for key in ("track_width", "axle_from_front", "width"):
        assert key in params["wheels"]
    assert params["body"]["length"] == 420
    assert params["body"]["width"] == 280
    assert "position_from_front" in params["lidar"]


def test_urdf_is_well_formed_and_has_the_frames(tree):
    links = {l.get("name") for l in tree.findall("link")}
    assert {"base_footprint", "base_link", "wheel_left", "wheel_right", "caster_link", "laser", "imu_link"} <= links
    assert tree.get("name") == "v8carrier"


def test_base_footprint_on_floor_under_axle(tree, params):
    xyz, j = joint_origin(tree, "base_footprint_joint")
    assert j.get("type") == "fixed"
    assert j.find("parent").get("link") == "base_footprint"
    assert xyz == pytest.approx([0.0, 0.0, params["wheels"]["diameter"] / 2000.0])


def test_wheels_on_the_axle_at_half_track(tree, params):
    half = params["wheels"]["track_width"] / 2000.0
    for name, sign in (("wheel_left_joint", 1), ("wheel_right_joint", -1)):
        xyz, j = joint_origin(tree, name)
        assert j.get("type") == "continuous"
        assert xyz == pytest.approx([0.0, sign * half, 0.0], abs=1e-4)
        assert j.find("axis").get("xyz") == "0 1 0"
    cyl = tree.find("./link[@name='wheel_left']/visual/geometry/cylinder")
    assert float(cyl.get("radius")) == pytest.approx(params["wheels"]["diameter"] / 2000.0)
    assert float(cyl.get("length")) == pytest.approx(params["wheels"]["width"] / 1000.0)


def test_body_box_size_and_placement(tree, params):
    L, W, H = (params["body"][k] / 1000.0 for k in ("length", "width", "height"))
    wheel_r = params["wheels"]["diameter"] / 2000.0
    box = tree.find("./link[@name='base_link']/visual/geometry/box")
    assert [float(v) for v in box.get("size").split()] == pytest.approx([L, W, H])
    origin = [float(v) for v in tree.find("./link[@name='base_link']/visual/origin").get("xyz").split()]
    # Front of the box is axle_from_front ahead of the axle.
    assert origin[0] + L / 2 == pytest.approx(params["wheels"]["axle_from_front"] / 1000.0, abs=1e-4)
    # Box bottom is the ground clearance above the floor.
    assert origin[2] - H / 2 + wheel_r == pytest.approx(G.BODY_GROUND_CLEARANCE_MM / 1000.0, abs=1e-4)


def test_laser_on_nose_tower(tree, params):
    xyz, _ = joint_origin(tree, "laser_joint")
    wheel_r = params["wheels"]["diameter"] / 2000.0
    expected_x = (params["wheels"]["axle_from_front"] - params["lidar"]["position_from_front"]) / 1000.0
    assert xyz[0] == pytest.approx(expected_x, abs=1e-4)
    assert xyz[1] == 0.0
    scan_plane_above_floor = xyz[2] + wheel_r
    assert scan_plane_above_floor == pytest.approx(
        (params["body"]["height"] + G.C1_SCAN_PLANE_ABOVE_BASE_MM) / 1000.0, abs=1e-4)


def test_caster_at_rear_on_floor(tree, params):
    xyz, _ = joint_origin(tree, "caster_joint")
    wheel_r = params["wheels"]["diameter"] / 2000.0
    rear = (params["wheels"]["axle_from_front"] - params["body"]["length"]) / 1000.0
    assert xyz[0] == pytest.approx(rear + params["caster"]["position_from_rear"] / 1000.0, abs=1e-4)
    ball_r = params["caster"]["ball_diameter"] / 2000.0
    assert xyz[2] + wheel_r == pytest.approx(ball_r, abs=1e-4)  # ball touches the floor


def test_imu_in_right_pod(tree, params):
    xyz, _ = joint_origin(tree, "imu_joint")
    assert xyz[1] < 0  # right side
    assert abs(xyz[1]) == pytest.approx((params["slot"]["width"] / 2 + params["pods"]["width"] / 2) / 1000.0,
                                        abs=1e-4)


def test_nav2_footprint_covers_body_and_wheels(params):
    fp = G.nav2_footprint(params)
    xs = [p[0] for p in fp]
    ys = [p[1] for p in fp]
    front = params["wheels"]["axle_from_front"] / 1000.0
    assert max(xs) == pytest.approx(front, abs=1e-3)
    assert min(xs) == pytest.approx(front - params["body"]["length"] / 1000.0, abs=1e-3)
    half_wheels = (params["wheels"]["track_width"] + params["wheels"]["width"]) / 2000.0
    half_body = params["body"]["width"] / 2000.0
    assert max(ys) == pytest.approx(max(half_wheels, half_body), abs=1e-3)
    assert min(ys) == -max(ys)
    f = G.frames_from_params(params)
    assert f["robot_radius"] >= math.hypot(max(xs), max(ys)) - 1e-3


def test_geometry_yaml_is_valid_ros_params(params):
    doc = yaml.safe_load(G.generate_geometry_yaml(params))
    p = doc["/**"]["ros__parameters"]
    assert p["wheel_diameter"] == pytest.approx(params["wheels"]["diameter"] / 1000.0)
    assert p["track_width"] == pytest.approx(params["wheels"]["track_width"] / 1000.0)
    assert p["brush_width"] == pytest.approx(params["brush_roll"]["length"] / 1000.0)
    assert yaml.safe_load(p["footprint"]) == G.nav2_footprint(params)


def test_committed_files_are_current(urdf, params):
    msg = "stale: run `python3 -m v8carrier_description.urdf_generator` after changing cad/params.yaml"
    with open(COMMITTED_URDF) as f:
        assert f.read() == urdf, "urdf/v8carrier.urdf " + msg
    with open(COMMITTED_GEOMETRY) as f:
        assert f.read() == G.generate_geometry_yaml(params), "config/geometry.yaml " + msg
