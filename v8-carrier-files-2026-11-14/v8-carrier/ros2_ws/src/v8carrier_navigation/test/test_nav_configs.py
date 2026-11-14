"""The hand-written Nav2 / slam_toolbox configs must agree with the generated
geometry and with the topics the bringup publishes."""

import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.normpath(os.path.join(HERE, "..", "config"))
GEOMETRY = os.path.normpath(os.path.join(HERE, "..", "..", "v8carrier_description", "config", "geometry.yaml"))


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)


def test_nav2_footprint_matches_generated_geometry():
    with open(GEOMETRY) as f:
        geometry = yaml.safe_load(f)["/**"]["ros__parameters"]
    nav2 = load("nav2_params.yaml")
    for costmap in ("local_costmap", "global_costmap"):
        fp = nav2[costmap][costmap]["ros__parameters"]["footprint"]
        assert yaml.safe_load(fp) == yaml.safe_load(geometry["footprint"]), costmap
        # Inflation radius clears the robot's own radius.
        inflation = nav2[costmap][costmap]["ros__parameters"]["inflation_layer"]["inflation_radius"]
        assert inflation > geometry["robot_radius"]


def test_velocity_limits_are_0_3_m_s():
    nav2 = load("nav2_params.yaml")
    dwb = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    assert dwb["max_vel_x"] == 0.3 and dwb["max_speed_xy"] == 0.3
    assert dwb["max_vel_y"] == 0.0  # differential drive
    vs = nav2["velocity_smoother"]["ros__parameters"]
    assert vs["max_velocity"][0] == 0.3 and vs["min_velocity"][0] == -0.3
    assert nav2["amcl"]["ros__parameters"]["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"


def test_scan_topics_use_the_filtered_scan():
    nav2 = load("nav2_params.yaml")
    for costmap in ("local_costmap", "global_costmap"):
        assert nav2[costmap][costmap]["ros__parameters"]["obstacle_layer"]["scan"]["topic"] == "/scan_filtered"
    assert nav2["amcl"]["ros__parameters"]["scan_topic"] == "scan_filtered"
    assert nav2["collision_monitor"]["ros__parameters"]["scan"]["topic"] == "scan_filtered"
    slam = load("slam_toolbox_online_async.yaml")["slam_toolbox"]["ros__parameters"]
    assert slam["scan_topic"] == "/scan_filtered"
    assert slam["base_frame"] == "base_footprint"
    assert slam["max_laser_range"] == 12.0  # RPLIDAR C1


def test_frames_are_consistent():
    nav2 = load("nav2_params.yaml")
    for section in ("bt_navigator", "behavior_server", "collision_monitor"):
        params = nav2[section]["ros__parameters"]
        frame = params.get("robot_base_frame") or params.get("base_frame_id")
        assert frame == "base_footprint", section


def test_coverage_config_row_spacing_is_brush_minus_overlap():
    cov = load("coverage.yaml")["coverage"]["ros__parameters"]
    assert cov["brush_width"] - cov["overlap"] == 0.22
    assert load("coverage.yaml")["mission"]["ros__parameters"]["vacuum_mode"] in ("soft", "hard")
