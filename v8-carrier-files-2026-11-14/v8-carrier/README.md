# V8 carrier robot

A 3D-printed carrier robot that docks an unmodified Dyson V8 Cyclone (SV55)
handheld vacuum and drives it around the home. Raspberry Pi 5 (4 GB) runs
ROS 2 Jazzy with Nav2; an ESP32-S3 handles motors, encoders, sensors and safety
stops; a 2D LiDAR maps the home. Printed on a Bambu Lab X1 Carbon (256 mm bed).

The build manual is the source of truth for decisions:
https://claude.ai/artifact/Vc65oKqf4N8fkRTHT9aa97

## Layout

| Path | Holds |
| --- | --- |
| `cad/` | Parametric CAD (build123d), one Python module per part under `cad/parts/`; `cad/measurements.yaml` (Stage 0 vacuum measurements) and `cad/params.yaml` (design parameters); exports in `cad/out/` |
| `firmware/` | ESP32-S3 firmware: motor PID, encoders, IMU, bumper, cliff, battery monitor, microphone level, servo, serial protocol, emergency stop |
| `ros2_ws/` | ROS 2 Jazzy packages: description, bringup, navigation |
| `docs/` | Reports: `fit-report.md`, `assembly.md` |
| `checks/` | Fit-check scripts run by the checker |

## Blocked until measured

The collar, cradle rails and cheeks, rear latch and servo mast depend on the
vacuum. They stay unbuilt until `cad/measurements.yaml` has real values for the
nose diameter and length, the body outline, the battery footprint and the power
button position.
