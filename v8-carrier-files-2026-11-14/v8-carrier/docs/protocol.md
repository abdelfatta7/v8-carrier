# Serial protocol: Raspberry Pi 5 <-> ESP32-S3

Shared by `firmware/lib/protocol/` (C++) and
`ros2_ws/src/v8carrier_bringup/v8carrier_bringup/protocol.py` (Python). Both are
checked against the same byte vectors in `docs/protocol_vectors.json`, which the
C++ test writes and pytest reads. If you change anything here, change both
implementations, regenerate the vectors
(`cmake --build firmware/build --target update_protocol_vectors`) and run both
test suites.

## Physical link

| Item | Value |
| --- | --- |
| Transport | UART, 3.3 V, 115200 baud, 8N1, no flow control |
| ESP32 side | UART1: TX = GPIO17, RX = GPIO18 (`firmware/src/pins.h`) |
| Pi side | GPIO14 TXD (header pin 8) -> ESP32 RX; GPIO15 RXD (pin 10) <- ESP32 TX; `/dev/ttyAMA0` |
| Pi setup | `enable_uart=1` in `/boot/firmware/config.txt`, serial console disabled (`raspi-config` > Interface > Serial: no login shell, hardware enabled) |

The ESP32's USB port is a debug console only (`pio device monitor`, 115200).
Using the Pi's GPIO UART keeps `/dev/ttyUSB0` for the RPLIDAR.

## Frame

```
offset  size  field
0       1     sync 0xAA
1       1     sync 0x55
2       1     id        message id (table below)
3       1     len       payload length, 0..64
4       len   payload
4+len   2     crc16     little-endian, over id, len and payload
```

CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no input or output
reflection, no final XOR. Check value: `crc("123456789") == 0x29B1`.

All multi-byte fields are little-endian. Integer types: `u8 i16 u16 i32 u32`.

The receiver scans for the sync pair, reads id and len, rejects `len > 64`,
collects the payload and checks the CRC. A bad CRC drops the frame and the scan
restarts at the next 0xAA. The sync pair may legitimately appear inside a
payload; the length field keeps the parser on track.

## Messages Pi -> ESP32 (ids < 0x80)

| id | name | payload | notes |
| --- | --- | --- | --- |
| 0x01 | SET_WHEEL_VEL | `i16 left_mm_s`, `i16 right_mm_s` | wheel rim speeds in mm/s, positive forward; clamped to +-600 in firmware; also counts as a heartbeat |
| 0x02 | SET_SERVO | `u8 angle_deg` 0..180 | manual servo position for calibration; refused (BUSY) while a vacuum button routine runs |
| 0x03 | REQUEST_STATUS | none | one STATUS frame now (useful when streaming is off) |
| 0x04 | HEARTBEAT | `u8 seq` | proves the Pi is alive; not acknowledged |
| 0x05 | CLEAR_ESTOP | none | clears the latch; refused (REJECTED_ESTOP) while any cause is still active |
| 0x06 | SET_VACUUM_MODE | `u8 mode` 0 off, 1 soft, 2 hard | starts the button-press routine; clears a previous vacuum fault first; BUSY if a routine is running |
| 0x07 | SET_BRUSH | `u8 duty_pct` 0..100 | brush roll motor; forced to 0 while the estop is latched |
| 0x08 | SET_STREAM | `u8 enable` 0/1 | turn the 50 Hz STATUS stream on/off (on at boot) |

Every command except HEARTBEAT is answered with an ACK.

## Messages ESP32 -> Pi (ids >= 0x80)

| id | name | payload |
| --- | --- | --- |
| 0x80 | STATUS | 47 bytes, layout below |
| 0x81 | ACK | `u8 id` (the command's id), `u8 result`: 0 OK, 1 REJECTED_ESTOP, 2 BAD_ARG, 3 BUSY |
| 0x82 | LOG | ASCII text (not NUL-terminated), up to 64 bytes |

### STATUS payload (47 bytes)

| offset | type | field | unit / meaning |
| --- | --- | --- | --- |
| 0 | u32 | t_ms | ESP32 `millis()` |
| 4 | i32 | left_ticks | cumulative left encoder ticks (wraps) |
| 8 | i32 | right_ticks | cumulative right encoder ticks (wraps) |
| 12 | i16 | gyro_x | mrad/s, body frame x forward, y left, z up |
| 14 | i16 | gyro_y | mrad/s |
| 16 | i16 | gyro_z | mrad/s (yaw rate) |
| 18 | i16 | accel_x | mm/s^2, including gravity |
| 20 | i16 | accel_y | mm/s^2 |
| 22 | i16 | accel_z | mm/s^2 |
| 24 | i16 | yaw | 1e-4 rad, fused heading (BNO085 game rotation vector); 0x7FFF = not available (MPU-6050) |
| 26 | u8 | bumper_bits | bit0 left, bit1 right; 1 = pressed |
| 27 | u16[4] | cliff_mv | sensor output in mV: nose-left, nose-right, wheel-left, wheel-right; low = no floor |
| 35 | u16 | pack_mv | pack voltage, mV |
| 37 | i16 | pack_ma | pack current, mA, positive = discharging |
| 39 | u16 | mic_level | microphone RMS over the last 64 ms window, 16-bit sample units |
| 41 | u8 | estop_flags | bit0 latched (motors cut); bit1 bumper, bit2 cliff, bit3 low battery, bit4 heartbeat lost = causes active right now |
| 42 | u8 | estop_latch_cause | the causes (same bits 1..4) seen since the latch set; 0 when not latched |
| 43 | u8 | servo_angle_deg | current servo command |
| 44 | u8 | vacuum_mode | 0 off, 1 soft, 2 hard, 3 unknown |
| 45 | u8 | vacuum_status | bits 0..6: 0 idle, 1 busy (press routine running), 2 fault (press not confirmed twice); bit7: mic says the vacuum is running |
| 46 | u8 | brush_duty_pct | 0..100 |

Encoder ticks: 1320 per wheel revolution by default (`ENCODER_TICKS_PER_REV`
in `pins.h`; 11 PPR x 4 edges x 1:30 gearbox, to be measured). Wheel diameter
80 mm and track width (333 mm as of 2026-11-10) are in `cad/params.yaml`; the Pi does
the odometry from the ticks with those numbers, the firmware only uses them for
its wheel-speed PIDs.

## Rates and timeouts

| Item | Value |
| --- | --- |
| STATUS stream | 50 Hz (every 20 ms), ~2.7 kB/s |
| Heartbeat from the Pi | 10 Hz recommended; SET_WHEEL_VEL frames also count |
| Heartbeat timeout | 500 ms without HEARTBEAT or SET_WHEEL_VEL -> estop latched, cause "heartbeat" |
| Wheel command timeout | 500 ms without SET_WHEEL_VEL -> wheel targets set to 0 (independent of the estop) |
| Control loop | 100 Hz |

## Emergency stop rules (firmware, no Pi needed)

Causes: bumper pressed (debounced 20 ms), cliff sensor below 900 mV for
30 ms, pack voltage below 14.0 V for 2 s (clears above 14.6 V), heartbeat
older than 500 ms. At boot the latch is set with cause "heartbeat" until the
Pi has sent a heartbeat and then CLEAR_ESTOP.

While latched: wheel motors brake, the brush stops, SET_WHEEL_VEL is answered
REJECTED_ESTOP. Exception: when the latch and the active causes are only
bumper and/or cliff, reverse commands (both wheels <= 0) are accepted, clamped
to 80 mm/s, so the robot can back away; the latch still needs CLEAR_ESTOP
afterwards. CLEAR_ESTOP succeeds only once every cause has cleared.

The Pi side (`serial_bridge`) clears a heartbeat-only latch by itself after it
reconnects; bumper/cliff latches are handled by its back-off recovery; a
low-battery latch is left to the operator.

## Vacuum button routine

One servo tap: press 300 ms, release, wait 1.5 s (spin-up plus the mic
detector's 0.5 s hold), compare the mic with the expected state. The V8's
single button cycles off -> soft -> hard -> off; SET_VACUUM_MODE taps the right
number of times. A tap that the mic does not confirm is retried once; a second
failure sets `vacuum_status = fault` and `vacuum_mode = unknown`. From unknown
the first tap is a discovery tap. Without a calibrated "hard" level
(`VacuumButtonConfig::hard_level`, from `cad/measurements.yaml` sound_levels)
the mic can only tell running from not running, so soft and hard are tracked
by counting taps.

If the vacuum goes quiet for 3 s while the firmware believes it is running, the
mode becomes off and a LOG "vacuum stopped by itself" is sent; the Pi's
battery-rules node reacts to the mode change on `/vacuum/state`.
