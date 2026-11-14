#include "../minitest.h"
#include "pid.h"

using v8c::motor_duty_limit;
using v8c::Pid;
using v8c::PidGains;

TEST(duty_limit_clamps_12v_motor_on_4s_pack) {
    CHECK_NEAR(motor_duty_limit(16.8f), 12.0f / 16.8f, 1e-6);
    CHECK_NEAR(motor_duty_limit(14.8f), 12.0f / 14.8f, 1e-6);
    CHECK_NEAR(motor_duty_limit(12.0f), 1.0f, 1e-6);
    CHECK_NEAR(motor_duty_limit(11.0f), 1.0f, 1e-6);
    CHECK_NEAR(motor_duty_limit(0.0f), 0.05f, 1e-6);  // nonsense reading -> crawl
}

TEST(proportional_only_output) {
    PidGains g;
    g.kp = 2.0f;
    Pid pid(g);
    CHECK_NEAR(pid.update(0.3f, 0.1f, 0.01f), 0.4f, 1e-6);
    CHECK_NEAR(pid.update(0.1f, 0.3f, 0.01f), -0.4f, 1e-6);
}

TEST(output_is_clamped_to_limit) {
    PidGains g;
    g.kp = 100.0f;
    g.output_limit = 1.0f;
    Pid pid(g);
    CHECK_NEAR(pid.update(1.0f, 0.0f, 0.01f), 1.0f, 1e-6);
    CHECK_NEAR(pid.update(-1.0f, 0.0f, 0.01f), -1.0f, 1e-6);
    pid.set_output_limit(12.0f / 16.8f);
    CHECK_NEAR(pid.update(1.0f, 0.0f, 0.01f), 12.0f / 16.8f, 1e-6);
}

TEST(integrator_does_not_wind_up_while_saturated) {
    PidGains g;
    g.kp = 1.0f;
    g.ki = 10.0f;
    g.integral_limit = 0.3f;
    g.output_limit = 1.0f;
    Pid pid(g);
    // Blocked wheel: large error for a long time.
    for (int i = 0; i < 1000; ++i) pid.update(1.0f, 0.0f, 0.01f);
    // The integral term contribution is bounded by integral_limit.
    CHECK(pid.integral() * g.ki <= g.integral_limit + 1e-6f);
    // Once the wheel reaches the setpoint the output drops quickly instead of
    // staying saturated for a long time: at most the integral term is left.
    const float out = pid.update(1.0f, 1.0f, 0.01f);
    CHECK(out <= g.integral_limit + 1e-6f);
    CHECK(out >= 0.0f);
}

TEST(integrator_removes_steady_state_error) {
    PidGains g;
    g.kp = 0.5f;
    g.ki = 5.0f;
    g.output_limit = 1.0f;
    Pid pid(g);
    // A plant whose speed is 0.5 * duty (so pure P leaves a steady-state error).
    float speed = 0.0f;
    float out = 0.0f;
    for (int i = 0; i < 2000; ++i) {
        out = pid.update(0.2f, speed, 0.01f);
        speed = 0.5f * out;
    }
    CHECK_NEAR(speed, 0.2f, 0.005f);
}

TEST(reset_clears_state) {
    PidGains g;
    g.kp = 1.0f;
    g.ki = 1.0f;
    g.kd = 0.1f;
    Pid pid(g);
    for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.0f, 0.01f);
    pid.reset();
    CHECK_NEAR(pid.integral(), 0.0f, 1e-9);
    CHECK_NEAR(pid.last_error(), 0.0f, 1e-9);
    // First update after reset has no derivative kick.
    CHECK_NEAR(pid.update(0.5f, 0.0f, 0.01f), 0.5f + 1.0f * 0.5f * 0.01f, 1e-6);
}

TEST(derivative_acts_on_error_change) {
    PidGains g;
    g.kd = 0.01f;
    Pid pid(g);
    CHECK_NEAR(pid.update(1.0f, 0.0f, 0.01f), 0.0f, 1e-6);  // no history yet
    CHECK_NEAR(pid.update(1.0f, 0.5f, 0.01f), 0.01f * (-0.5f) / 0.01f, 1e-6);
}

TEST(feedforward_adds_duty_per_setpoint) {
    PidGains g;
    g.kff = 2.0f;
    Pid pid(g);
    CHECK_NEAR(pid.update(0.3f, 0.3f, 0.01f), 0.6f, 1e-6);
}

MINITEST_MAIN("pid")
