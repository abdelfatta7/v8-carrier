// Per-wheel velocity PID with anti-windup and output clamp.
// Plain C++17, no Arduino includes, so it is unit-tested on the host.
#pragma once

namespace v8c {

struct PidGains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    // |integral term contribution| is clamped to this value (same units as the
    // output, i.e. PWM duty 0..1). Stops the integrator winding up while the
    // wheel is blocked or the output is saturated.
    float integral_limit = 0.5f;
    // Absolute clamp on the output. The caller usually lowers this further with
    // motor_duty_limit() so a 12 V motor is not overdriven on a 4S pack.
    float output_limit = 1.0f;
    // Feed-forward: duty per unit of setpoint (e.g. duty per m/s). Optional,
    // gives the PID a head start so the integrator does less work.
    float kff = 0.0f;
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidGains& g) : gains_(g) {}

    void set_gains(const PidGains& g) { gains_ = g; }
    const PidGains& gains() const { return gains_; }

    // Lower (or raise) the output clamp at run time, e.g. from the pack voltage.
    void set_output_limit(float limit);
    float output_limit() const { return gains_.output_limit; }

    // One controller step. dt in seconds. Returns the clamped output.
    float update(float setpoint, float measured, float dt);

    // Forget the integrator and the derivative history (e.g. after an estop).
    void reset();

    float integral() const { return integral_; }
    float last_error() const { return last_error_; }

private:
    PidGains gains_;
    float integral_ = 0.0f;
    float last_error_ = 0.0f;
    bool has_last_ = false;
};

// Duty limit so that the average motor voltage never exceeds motor_rated_v.
// For a 12 V motor on a 4S pack (14.8 - 16.8 V) that is 12.0 / pack_voltage.
// Returns 1.0 when the pack is at or below the motor's rated voltage, and a
// small positive floor if the pack reading is nonsense (zero / negative).
float motor_duty_limit(float pack_voltage_v, float motor_rated_v = 12.0f);

}  // namespace v8c
