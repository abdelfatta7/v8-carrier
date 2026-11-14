// V8 carrier robot: ESP32-S3 firmware, hardware layer.
//
// Everything that can be tested without hardware lives in lib/<module>/ (PID,
// odometry, protocol, safety, vacuum button, mic level) and is exercised by
// test/. This file wires those modules to the pins in pins.h and runs the
// loops:
//   100 Hz  read sensors -> safety -> wheel PID -> motors; vacuum button tick
//    50 Hz  STATUS frame to the Pi
//   always  parse frames from the Pi
// The Pi link is UART1 (pins.h); the USB port stays a debug console.
//
// Build: PlatformIO (platformio.ini). Targets Arduino-ESP32 core 3.x; the LEDC
// calls also compile on 2.x through the version guards in motor.h/servo_ledc.h.

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "encoder.h"
#include "imu.h"
#include "ina226.h"
#include "mic_i2s.h"
#include "miclevel.h"
#include "motor.h"
#include "odometry.h"
#include "pid.h"
#include "pins.h"
#include "protocol.h"
#include "safety.h"
#include "servo_ledc.h"
#include "vacuum_button.h"

using namespace v8c;

// ---- Tunables ---------------------------------------------------------------
constexpr uint32_t kControlPeriodMs = 10;   // 100 Hz control loop
constexpr uint32_t kStatusPeriodMs = 20;    // 50 Hz telemetry
constexpr uint32_t kCmdTimeoutMs = 500;     // SET_WHEEL_VEL older than this -> wheels stop
constexpr uint32_t kBatteryPeriodMs = 50;   // INA226 read rate
constexpr float kMotorRatedV = 12.0f;       // JGB37-520 rating; duty clamp = 12 / pack V
constexpr float kMaxWheelSpeed = 0.6f;      // m/s, hard clamp on wheel targets
constexpr float kBackOffSpeed = 0.08f;      // m/s, max reverse speed while latched by bumper/cliff

// Wheel PID. Output is duty "as a fraction of 12 V"; it is scaled by
// motor_duty_limit(pack V) before it reaches the H-bridge so the gains do not
// change with the pack's state of charge. kff is 1 / (free speed at 12 V):
// JGB37-520 1:30 does about 330 rpm -> 1.4 m/s on 80 mm wheels. Measure.
static PidGains wheel_gains() {
    PidGains g;
    g.kp = 1.2f;
    g.ki = 6.0f;
    g.kd = 0.0f;
    g.kff = 0.7f;
    g.integral_limit = 0.4f;
    g.output_limit = 1.0f;
    return g;
}

// ---- Hardware ---------------------------------------------------------------
#ifdef IMU_DRIVER_MPU6050
static Mpu6050Imu g_imu_driver(MPU6050_ADDR);
#else
static Bno085Imu g_imu_driver(BNO085_ADDR);
#endif
static ImuDriver& g_imu = g_imu_driver;

static Drv8871 g_motor_l, g_motor_r, g_brush;
static QuadratureEncoder g_enc_l, g_enc_r;
static Ina226 g_ina;
static MicI2s g_mic;
static ServoLedc g_servo;

// ---- Logic ------------------------------------------------------------------
static Odometry g_odom;
static Pid g_pid_l, g_pid_r;
static SafetyMonitor g_safety;
static VacuumButton g_vacuum;
static proto::FrameParser g_parser;

// ---- State ------------------------------------------------------------------
static float g_target_l = 0.0f, g_target_r = 0.0f;  // m/s
static uint32_t g_last_cmd_ms = 0;
static bool g_stream = true;
static uint8_t g_brush_duty = 0;
static uint8_t g_manual_servo = 255;  // 255 = none; SET_SERVO override while the vacuum routine is idle
static ImuSample g_imu_sample;
static bool g_imu_ok = false;
static float g_pack_v = 0.0f, g_pack_a = 0.0f;
static proto::StatusMsg g_status;
static uint32_t g_next_control_ms = 0, g_next_status_ms = 0, g_next_battery_ms = 0;

// ---- Link helpers -----------------------------------------------------------
static void send_frame(uint8_t id, const uint8_t* payload, size_t len) {
    uint8_t buf[proto::kMaxFrame];
    const size_t n = proto::encode_frame(id, payload, len, buf, sizeof buf);
    if (n) Serial1.write(buf, n);
}

static void send_ack(uint8_t id, uint8_t result) {
    proto::AckMsg a;
    a.id = id;
    a.result = result;
    uint8_t p[proto::AckMsg::kSize];
    send_frame(proto::ACK, p, a.pack(p));
}

static void send_log(const char* text) {
    size_t n = strlen(text);
    if (n > proto::kMaxPayload) n = proto::kMaxPayload;
    send_frame(proto::LOG, reinterpret_cast<const uint8_t*>(text), n);
    Serial.println(text);
}

static int16_t clamp_i16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return static_cast<int16_t>(lroundf(v));
}

// ---- Command handling -------------------------------------------------------
static void handle_frame(uint32_t now) {
    const uint8_t id = g_parser.id();
    const uint8_t* p = g_parser.payload();
    const size_t n = g_parser.length();

    switch (id) {
        case proto::SET_WHEEL_VEL: {
            proto::SetWheelVelMsg m;
            if (!m.unpack(p, n)) { send_ack(id, proto::ACK_BAD_ARG); return; }
            float l = m.left_mm_s * 1e-3f, r = m.right_mm_s * 1e-3f;
            l = constrain(l, -kMaxWheelSpeed, kMaxWheelSpeed);
            r = constrain(r, -kMaxWheelSpeed, kMaxWheelSpeed);
            g_safety.heartbeat(now);  // a velocity command proves the Pi is alive
            g_last_cmd_ms = now;
            if (g_safety.motion_allowed(l, r)) {
                if (g_safety.latched()) {  // back-off exception: keep it slow
                    l = constrain(l, -kBackOffSpeed, 0.0f);
                    r = constrain(r, -kBackOffSpeed, 0.0f);
                }
                g_target_l = l;
                g_target_r = r;
                send_ack(id, proto::ACK_OK);
            } else {
                g_target_l = g_target_r = 0.0f;
                send_ack(id, proto::ACK_REJECTED_ESTOP);
            }
            return;
        }
        case proto::SET_SERVO: {
            proto::SetServoMsg m;
            if (!m.unpack(p, n) || m.angle_deg > 180) { send_ack(id, proto::ACK_BAD_ARG); return; }
            if (g_vacuum.busy()) { send_ack(id, proto::ACK_BUSY); return; }
            g_manual_servo = m.angle_deg;
            send_ack(id, proto::ACK_OK);
            return;
        }
        case proto::REQUEST_STATUS:
            g_next_status_ms = now;  // send on the next loop pass
            send_ack(id, proto::ACK_OK);
            return;
        case proto::HEARTBEAT: {
            proto::HeartbeatMsg m;
            if (!m.unpack(p, n)) { send_ack(id, proto::ACK_BAD_ARG); return; }
            g_safety.heartbeat(now);
            return;  // no ack: it would double the idle traffic for nothing
        }
        case proto::CLEAR_ESTOP:
            if (g_safety.clear()) {
                g_pid_l.reset();
                g_pid_r.reset();
                send_ack(id, proto::ACK_OK);
            } else {
                send_ack(id, proto::ACK_REJECTED_ESTOP);
            }
            return;
        case proto::SET_VACUUM_MODE: {
            proto::SetVacuumModeMsg m;
            if (!m.unpack(p, n) || m.mode > proto::VAC_HARD) { send_ack(id, proto::ACK_BAD_ARG); return; }
            g_vacuum.clear_fault();  // a new request is the operator's retry
            g_manual_servo = 255;
            send_ack(id, g_vacuum.request(static_cast<VacMode>(m.mode)) ? proto::ACK_OK : proto::ACK_BUSY);
            return;
        }
        case proto::SET_BRUSH: {
            proto::SetBrushMsg m;
            if (!m.unpack(p, n) || m.duty_pct > 100) { send_ack(id, proto::ACK_BAD_ARG); return; }
            g_brush_duty = m.duty_pct;
            send_ack(id, proto::ACK_OK);
            return;
        }
        case proto::SET_STREAM: {
            proto::SetStreamMsg m;
            if (!m.unpack(p, n)) { send_ack(id, proto::ACK_BAD_ARG); return; }
            g_stream = m.enable != 0;
            send_ack(id, proto::ACK_OK);
            return;
        }
        default:
            send_ack(id, proto::ACK_BAD_ARG);
            return;
    }
}

// ---- Sensors ----------------------------------------------------------------
static uint8_t read_bumpers() {
    uint8_t bits = 0;
    if (digitalRead(PIN_BUMPER_L) == LOW) bits |= 0x01;
    if (digitalRead(PIN_BUMPER_R) == LOW) bits |= 0x02;
    return bits;
}

static void read_cliffs(uint16_t out[4]) {
    static const int pins[4] = {PIN_CLIFF_0, PIN_CLIFF_1, PIN_CLIFF_2, PIN_CLIFF_3};
    for (int k = 0; k < 4; ++k) out[k] = static_cast<uint16_t>(analogReadMilliVolts(pins[k]));
}

static void read_battery() {
    float v, a;
    if (g_ina.ok() && g_ina.read_voltage(v) && g_ina.read_current(a)) {
        g_pack_v = v;
        g_pack_a = a;
    }
}

// ---- Control loop -----------------------------------------------------------
static void control_step(uint32_t now) {
    const float dt = kControlPeriodMs * 1e-3f;

    // Encoders -> odometry (gives wheel speeds for the PIDs).
    const int32_t tl = g_enc_l.read(), tr = g_enc_r.read();
    g_odom.update(tl, tr, dt);

    // IMU.
    ImuSample s;
    if (g_imu.read(s)) {
        g_imu_sample = s;
        g_imu_ok = true;
    }

    // Safety.
    SafetyInputs in;
    in.bumper_bits = read_bumpers();
    read_cliffs(in.cliff_mv);
    in.pack_v = g_pack_v;
    g_safety.update(now, in);

    // Wheel command timeout.
    if (now - g_last_cmd_ms > kCmdTimeoutMs) g_target_l = g_target_r = 0.0f;

    // Wheel PIDs -> H-bridges.
    const float duty_limit = motor_duty_limit(g_pack_v, kMotorRatedV);
    if (g_safety.motion_allowed(g_target_l, g_target_r) && (g_target_l != 0.0f || g_target_r != 0.0f)) {
        const float out_l = g_pid_l.update(g_target_l, g_odom.left_speed(), dt);
        const float out_r = g_pid_r.update(g_target_r, g_odom.right_speed(), dt);
        g_motor_l.set(out_l * duty_limit);
        g_motor_r.set(out_r * duty_limit);
    } else {
        g_pid_l.reset();
        g_pid_r.reset();
        g_motor_l.brake();
        g_motor_r.brake();
    }

    // Brush roll: on only while not latched.
    g_brush.set(g_safety.latched() ? 0.0f : g_brush_duty / 100.0f * duty_limit);

    // Vacuum button routine, then the servo.
    g_vacuum.tick(now, g_mic.running(), g_mic.level());
    if (g_vacuum.busy() || g_manual_servo == 255)
        g_servo.write_deg(g_vacuum.servo_angle());
    else
        g_servo.write_deg(g_manual_servo);

    if (g_vacuum.take_self_stop_event()) send_log("vacuum stopped by itself");

    // Snapshot for the status message.
    g_status.left_ticks = tl;
    g_status.right_ticks = tr;
    for (int k = 0; k < 3; ++k) {
        g_status.gyro_mrad_s[k] = clamp_i16(g_imu_sample.gyro[k] * 1000.0f);
        g_status.accel_mm_s2[k] = clamp_i16(g_imu_sample.accel[k] * 1000.0f);
    }
    g_status.yaw_1e4_rad = (g_imu_ok && !std::isnan(g_imu_sample.yaw)) ? clamp_i16(g_imu_sample.yaw * 10000.0f) : 0x7FFF;
    g_status.bumper_bits = in.bumper_bits;
    for (int k = 0; k < 4; ++k) g_status.cliff_mv[k] = in.cliff_mv[k];
    g_status.pack_mv = static_cast<uint16_t>(constrain(g_pack_v * 1000.0f, 0.0f, 65535.0f));
    g_status.pack_ma = clamp_i16(g_pack_a * 1000.0f);
    g_status.mic_level = g_mic.level();
    g_status.estop_flags = g_safety.flags();
    g_status.estop_latch_cause = g_safety.latch_cause();
    g_status.servo_angle_deg = g_servo.last_deg();
    g_status.vacuum_mode = static_cast<uint8_t>(g_vacuum.mode());
    g_status.vacuum_status = static_cast<uint8_t>(g_vacuum.status()) | (g_mic.running() ? proto::kVacuumStatusMicRunning : 0);
    g_status.brush_duty_pct = g_safety.latched() ? 0 : g_brush_duty;
}

static void send_status(uint32_t now) {
    g_status.t_ms = now;
    uint8_t p[proto::StatusMsg::kSize];
    send_frame(proto::STATUS, p, g_status.pack(p));
}

// ---- Arduino entry points ---------------------------------------------------
void setup() {
    Serial.begin(115200);  // USB debug console
    Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
    pinMode(PIN_LED, OUTPUT);

    // Motors and encoders.
    g_motor_l.begin(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, MOTOR_L_INVERT, 0);
    g_motor_r.begin(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, MOTOR_R_INVERT, 2);
    g_brush.begin(PIN_BRUSH_IN1, PIN_BRUSH_IN2, false, 4);
    g_enc_l.begin(PIN_ENC_L_A, PIN_ENC_L_B, ENC_L_INVERT);
    g_enc_r.begin(PIN_ENC_R_A, PIN_ENC_R_B, ENC_R_INVERT);

    // The firmware only uses the per-wheel speeds from the odometry (for the
    // PIDs); the Pi computes the pose from the raw ticks with the geometry in
    // cad/params.yaml, so the track width here does not affect anything.
    OdomParams op;
    op.ticks_per_rev = ENCODER_TICKS_PER_REV;
    op.wheel_diameter_m = 0.080f;  // cad/params.yaml wheels.diameter
    op.track_width_m = 0.333f;     // cad/params.yaml wheels.track_width (derived 2026-11-10)
    g_odom.set_params(op);
    g_pid_l.set_gains(wheel_gains());
    g_pid_r.set_gains(wheel_gains());

    // Switches and analog inputs.
    pinMode(PIN_BUMPER_L, INPUT_PULLUP);
    pinMode(PIN_BUMPER_R, INPUT_PULLUP);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0 .. ~3.1 V

    // I2C devices.
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    if (!g_ina.begin(Wire, INA226_ADDR, INA226_SHUNT_OHMS)) send_log("INA226 not found");
    if (!g_imu.begin(Wire)) send_log("IMU not found");
    else send_log(g_imu.name());

    // Microphone.
    MicDetectorConfig mc;  // defaults in miclevel.h; tune from measurements.yaml sound_levels
    if (!g_mic.begin(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DIN, MIC_SAMPLE_RATE, MIC_WINDOW_SAMPLES, mc))
        send_log("I2S mic failed");

    // Servo and the vacuum button routine.
    VacuumButtonConfig vc;  // defaults in vacuum_button.h; rest/press angles need the servo mast
    g_vacuum.set_config(vc);
    g_servo.begin(PIN_SERVO, 6, SERVO_MIN_US, SERVO_MAX_US);
    g_servo.write_deg(vc.rest_angle_deg);

    SafetyConfig sc;  // defaults in safety.h
    g_safety.set_config(sc);

    send_log("v8carrier firmware up");
}

void loop() {
    const uint32_t now = millis();

    // Frames from the Pi.
    while (Serial1.available()) {
        if (g_parser.feed(static_cast<uint8_t>(Serial1.read()))) handle_frame(now);
    }

    if (static_cast<int32_t>(now - g_next_battery_ms) >= 0) {
        g_next_battery_ms = now + kBatteryPeriodMs;
        read_battery();
    }

    if (static_cast<int32_t>(now - g_next_control_ms) >= 0) {
        g_next_control_ms += kControlPeriodMs;
        if (static_cast<int32_t>(now - g_next_control_ms) > 0) g_next_control_ms = now + kControlPeriodMs;  // fell behind
        control_step(now);
    }

    if (g_stream && static_cast<int32_t>(now - g_next_status_ms) >= 0) {
        g_next_status_ms = now + kStatusPeriodMs;
        send_status(now);
        digitalWrite(PIN_LED, g_safety.latched() ? (now / 250) % 2 : (now / 1000) % 2);
    }
}
