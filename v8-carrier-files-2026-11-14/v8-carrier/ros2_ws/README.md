# ROS 2 workspace (Jazzy)

Three `ament_python` packages:

| Package | Holds |
| --- | --- |
| `v8carrier_description` | `urdf_generator.py` writes `urdf/v8carrier.urdf` and `config/geometry.yaml` from `cad/params.yaml`; `description.launch.py` runs `robot_state_publisher` |
| `v8carrier_bringup` | `serial_bridge` (ESP32 link, `docs/protocol.md`), `scan_filter` (blanks the rear 100 degrees the vacuum blocks), `battery_rules` (go home on low pack / vacuum self-stop), `web_ui` (Start/Stop/Go home/Schedule page on port 8080), `config/ekf.yaml` (robot_localization), `config/sllidar_c1.yaml`, `bringup.launch.py`, `lidar.launch.py` |
| `v8carrier_navigation` | `config/slam_toolbox_online_async.yaml`, `config/nav2_params.yaml` (DWB, 0.3 m/s, footprint from the body), `coverage` (boustrophedon rows 0.22 m apart over the map, sent to Nav2 FollowWaypoints), `mission` (vacuum on, cover, vacuum off, return), `slam.launch.py`, `navigation.launch.py`, `coverage.launch.py` |

Pure logic lives in importable modules with no ROS dependency and is covered
by pytest: `protocol.py`, `odometry.py`, `sector_blank.py`, `battery_rules.py`,
`recovery.py` (bringup); `urdf_generator.py` (description);
`coverage_planner.py`, `map_io.py`, `mission_fsm.py` (navigation). The nodes
import `rclpy` inside a guarded block and only run on the robot.

## Build and test without ROS (what CI and the host run)

```sh
pip install --break-system-packages colcon-common-extensions colcon-ros setuptools empy lark pytest pyserial numpy pyyaml
cd ros2_ws
colcon build            # works without ROS: ament_python packages only need setuptools
pytest src              # or, from the repo root: pytest ros2_ws/src
```

`colcon build` here produces `install/` with the launch files, configs, URDF
and console scripts; nothing is executable without `rclpy`, but the build
proves the packaging. The protocol test replays `docs/protocol_vectors.json`,
which the C++ firmware test writes (`firmware/README.md`), so the two sides
cannot drift apart silently. `test_urdf_generator.py` fails when
`urdf/v8carrier.urdf` or `config/geometry.yaml` is older than
`cad/params.yaml`; regenerate with

```sh
cd ros2_ws/src/v8carrier_description && python3 -m v8carrier_description.urdf_generator
```

and copy the new `footprint` line into `v8carrier_navigation/config/nav2_params.yaml`
(`test_nav_configs.py` checks that they match).

## Install on the Raspberry Pi 5 (Ubuntu 24.04)

```sh
# ROS 2 Jazzy, base install (no GUI on the Pi)
sudo apt install software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
    python3-serial python3-numpy python3-yaml python3-colcon-common-extensions

# RPLIDAR driver (not in apt)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git

# This workspace: copy or clone the repository, then symlink the packages in
ln -s /path/to/repo/ros2_ws/src/v8carrier_description .
ln -s /path/to/repo/ros2_ws/src/v8carrier_bringup .
ln -s /path/to/repo/ros2_ws/src/v8carrier_navigation .

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Serial ports:

* ESP32 link: the Pi's GPIO UART (`/dev/ttyAMA0`). In `/boot/firmware/config.txt`
  add `enable_uart=1` and `dtoverlay=disable-bt` (so the full UART is on
  GPIO14/15), disable the serial console (`sudo raspi-config` > Interface
  Options > Serial Port: login shell no, hardware yes), and add your user to
  `dialout`. Wiring is in `docs/protocol.md`.
* RPLIDAR C1: USB, `/dev/ttyUSB0`. A udev rule keeps the name stable:
  `SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"`
  then `lidar_port:=/dev/rplidar`.

## Run

Every launch below assumes `source ~/ros2_ws/install/setup.bash`.

### 1. Bringup (always)

```sh
ros2 launch v8carrier_bringup bringup.launch.py           # port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0 use_ekf:=true
```

Starts robot_state_publisher, the serial bridge, the LiDAR, the scan filter,
the EKF, the battery rules and the web UI (`http://<pi>:8080/`). Check:

```sh
ros2 topic echo /estop --once         # should clear itself after the bridge connects
ros2 topic hz /odom /imu/data /scan_filtered
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive by hand (apt: ros-jazzy-teleop-twist-keyboard)
```

### 2. Mapping

```sh
ros2 launch v8carrier_navigation slam.launch.py
```

Drive the robot around every room (teleop, or Nav2 goals from RViz on a
laptop: `ros2 launch nav2_bringup rviz_launch.py`). Save the map:

```sh
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/home
```

### 3. Navigation on the saved map

```sh
ros2 launch v8carrier_navigation navigation.launch.py map:=$HOME/maps/home.yaml
```

AMCL starts at the map origin; set the initial pose in RViz ("2D Pose
Estimate") or start the robot where the map was started.

### 4. Coverage and missions

```sh
ros2 launch v8carrier_navigation coverage.launch.py
```

Then press Start on the web page, or:

```sh
ros2 service call /mission/start std_srvs/srv/Trigger
ros2 topic echo /mission/state
ros2 topic pub --once /mission/schedule std_msgs/msg/String "{data: '09:30'}"   # daily; '' clears
```

A mission: records the start pose, switches the vacuum to `soft` (the servo
taps the button, the microphone confirms), starts the brush, plans
boustrophedon rows 0.22 m apart over the free space of the map and sends them
to Nav2's FollowWaypoints, then switches the vacuum off and navigates back
to the start. Stop halts everything where it stands; Go home stops the vacuum
and returns. The battery rules send Go home when the pack stays under 14.4 V
for 10 s or the vacuum stops on its own (its battery flat or its bin full).

## Topics and services

| Name | Type | Who |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 -> serial_bridge |
| `/odom` | nav_msgs/Odometry | serial_bridge (TF only when `use_ekf:=false`) |
| `/imu/data` | sensor_msgs/Imu | serial_bridge |
| `/battery_state` | sensor_msgs/BatteryState | serial_bridge, 5 Hz |
| `/bumper` | std_msgs/UInt8 (bit0 left, bit1 right) | serial_bridge, on change |
| `/cliff` | std_msgs/UInt16MultiArray (mV x4) | serial_bridge, 5 Hz |
| `/vacuum/state` | std_msgs/String JSON `{mode, status, mic_running, mic_level, ...}` | serial_bridge, 5 Hz |
| `/estop` | std_msgs/String JSON `{latched, active, latch_causes, recovery}` | serial_bridge, 5 Hz |
| `/vacuum/set_mode`, `/brush/set`, `/servo/set` | std_msgs/UInt8 | -> serial_bridge |
| `/estop/clear` | std_srvs/Trigger | serial_bridge |
| `/scan` -> `/scan_filtered` | sensor_msgs/LaserScan | sllidar_node -> scan_filter |
| `/mission/start`, `/mission/stop`, `/mission/go_home` | std_srvs/Trigger | mission |
| `/mission/command` | std_msgs/String (start, stop, go_home) | battery_rules / anyone -> mission |
| `/mission/schedule` | std_msgs/String "HH:MM" | web_ui -> mission |
| `/mission/state` | std_msgs/String JSON | mission, 1 Hz |
| `/coverage/start`, `/coverage/cancel` | std_srvs/Trigger | coverage |
| `/coverage/status` | std_msgs/String JSON | coverage |
| `/coverage/path` | nav_msgs/Path | coverage (RViz) |

## Decisions the owner should know

* The serial bridge auto-clears an estop that was latched only because the Pi
  was away (heartbeat), and backs away 1 s at 60 mm/s from a bumper or cliff
  latch (up to 3 attempts) before clearing it. A low-battery latch is never
  cleared automatically. `bridge.yaml` turns these off.
* The EKF uses wheel velocities and the gyro yaw rate only. The BNO085's fused
  heading is published in `/imu/data` but not fused.
* The URDF's ground clearance (20 mm), LiDAR scan-plane height (body top +
  27 mm) and IMU position are placeholders in `urdf_generator.py` until the
  CAD decides them.
* Nav2 uses the footprint polygon `[[0.19, 0.179], ...]`: body length 420 mm
  with the axle 190 mm from the front, width to the outside of the wheels
  (track 333 + 25 mm). Inflation radius 0.45 m; coverage rows keep 0.291 +
  0.03 m from obstacles, so narrow gaps under ~0.65 m are not swept.
* Unknown map cells count as obstacles for coverage, which keeps the robot
  inside the mapped rooms.
