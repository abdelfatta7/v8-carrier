#include "pid.h"

namespace v8c {

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void Pid::set_output_limit(float limit) {
    if (limit < 0.0f) limit = 0.0f;
    gains_.output_limit = limit;
}

float Pid::update(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;
    const float error = setpoint - measured;

    const float p = gains_.kp * error;
    const float ff = gains_.kff * setpoint;

    float d = 0.0f;
    if (has_last_) d = gains_.kd * (error - last_error_) / dt;
    last_error_ = error;
    has_last_ = true;

    // Provisional output without the new integral contribution.
    const float unsat = p + d + ff + gains_.ki * integral_;
    const float limit = gains_.output_limit;

    // Conditional integration: only integrate when the output is not already
    // saturated in the direction the error is pushing it. Then clamp the
    // integral term itself as a second line of defence.
    const bool saturated_same_sign =
        (unsat >= limit && error > 0.0f) || (unsat <= -limit && error < 0.0f);
    if (!saturated_same_sign && gains_.ki != 0.0f) {
        integral_ += error * dt;
        const float i_max = gains_.integral_limit / gains_.ki;
        integral_ = clampf(integral_, -i_max, i_max);
    }

    const float out = p + d + ff + gains_.ki * integral_;
    return clampf(out, -limit, limit);
}

void Pid::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    has_last_ = false;
}

float motor_duty_limit(float pack_voltage_v, float motor_rated_v) {
    if (pack_voltage_v <= 0.1f || motor_rated_v <= 0.0f) return 0.05f;  // nonsense reading: crawl only
    if (pack_voltage_v <= motor_rated_v) return 1.0f;
    return motor_rated_v / pack_voltage_v;
}

}  // namespace v8c
