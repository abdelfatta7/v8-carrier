# Firmware (ESP32-S3 DevKitC-1, Arduino framework, PlatformIO)

The ESP32 owns everything that must keep working when the Pi is slow or gone:
motor PWM and wheel-speed PIDs, encoders, IMU, bumpers, cliff sensors, pack
monitor, microphone level, the servo that taps the Dyson's button, and the
emergency stop. The Pi talks to it over UART with the protocol in
[`docs/protocol.md`](../docs/protocol.md).

## Layout

| Path | Holds |
| --- | --- |
| `platformio.ini` | `esp32-s3-devkitc-1` env (the robot) and `native` env (host tests) |
| `src/pins.h` | the pin map and the hardware constants (encoder ticks/rev, shunt, servo pulse range) |
| `src/main.cpp` | the hardware layer: 100 Hz control loop, 50 Hz telemetry, frame handling |
| `src/motor.h` | DRV8871 on LEDC PWM (slow-decay drive, brake, coast) |
| `src/encoder.h` | quadrature decode in GPIO interrupts |
| `src/imu.h` | `ImuDriver` interface; `Bno085Imu` (Adafruit BNO08x library) and `Mpu6050Imu` (raw registers) |
| `src/ina226.h` | INA226 pack voltage/current, raw registers |
| `src/mic_i2s.h` | INMP441 over I2S in a FreeRTOS task, feeds `miclevel` |
| `src/servo_ledc.h` | hobby servo on LEDC 50 Hz |
| `lib/pid/` | per-wheel velocity PID, anti-windup, output clamp, `motor_duty_limit()` = 12 V / pack V |
| `lib/odometry/` | ticks -> wheel distance -> differential-drive pose and wheel speeds |
| `lib/protocol/` | CRC-16, framing, message pack/unpack |
| `lib/safety/` | the latched emergency stop (bumper, cliff, low pack, heartbeat) |
| `lib/vacuum_button/` | the one-button off/soft/hard state machine and the press-confirm-retry routine |
| `lib/miclevel/` | RMS window and the running detector (adaptive threshold, hysteresis, 0.5 s hold) |
| `test/` | one assert-based test executable per lib module (`test/minitest.h`, no framework) |
| `CMakeLists.txt` | host build of lib/ + test/ with CTest |

`lib/` has no Arduino includes and is what the tests cover. `src/` is the only
code that needs the hardware.

## Build and test on the host (no toolchain needed)

```sh
cd firmware
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

Seven CTest entries: pid, odometry, safety, vacuum_button, miclevel, protocol,
and `protocol_vectors_match_docs`, which fails if `docs/protocol_vectors.json`
no longer matches what the C++ encoder produces. After a deliberate protocol
change run `cmake --build build --target update_protocol_vectors` and then the
Python tests in `ros2_ws/`.

With PlatformIO installed, `pio test -e native` runs the same test sources.

## Build for the robot

```sh
pio run -e esp32-s3-devkitc-1 -t upload
pio device monitor      # USB debug console, 115200
```

`platformio.ini` points at the pioarduino platform (Arduino-ESP32 core 3.x);
the official `espressif32` platform (core 2.0.x) also compiles it, see the
comment there. The BNO085 driver needs the Adafruit BNO08x library from
`lib_deps`; with `-D IMU_DRIVER_MPU6050` in `build_flags` the MPU-6050 driver
is used instead and no library is needed.

This firmware has not yet been compiled for the target in this repository's
history (the toolchain was not reachable where it was written); the host tests
and a syntax pass against stub Arduino headers are what it has been through.
Expect the first `pio run` to need small fixes in `src/`.

## Defaults the owner should check

| What | Default | Where |
| --- | --- | --- |
| Pin map | see `src/pins.h` | `src/pins.h` |
| Encoder ticks per wheel rev | 1320 (11 PPR x 4 x 1:30) | `ENCODER_TICKS_PER_REV` |
| Right encoder inverted | yes (mirrored motor) | `ENC_R_INVERT` |
| INA226 shunt | 10 mOhm | `INA226_SHUNT_OHMS` |
| Motor voltage clamp | duty <= 12.0 / pack V | `kMotorRatedV` in `main.cpp` |
| Wheel PID | kp 1.2, ki 6, kd 0, kff 0.7 | `wheel_gains()` in `main.cpp` |
| Low battery estop | < 14.0 V for 2 s, clears > 14.6 V | `SafetyConfig` in `lib/safety/safety.h` |
| Cliff "no floor" | < 900 mV for 3 samples | `SafetyConfig` |
| Heartbeat timeout | 500 ms | `SafetyConfig` |
| Mic running detector | on > max(1500, 4 x floor), off at 60 %, 0.5 s hold | `MicDetectorConfig` in `lib/miclevel/miclevel.h` |
| Servo rest / press angle | 20 / 70 deg | `VacuumButtonConfig` in `lib/vacuum_button/vacuum_button.h` |
| Press / confirm times | 300 ms / 1.5 s, retry once | `VacuumButtonConfig` |
| Vacuum self-stop | quiet 3 s | `VacuumButtonConfig` |
