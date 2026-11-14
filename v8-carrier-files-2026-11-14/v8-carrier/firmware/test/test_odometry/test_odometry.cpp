#include <cmath>

#include "../minitest.h"
#include "odometry.h"

using v8c::OdomParams;
using v8c::Odometry;

static OdomParams params() {
    OdomParams p;
    p.ticks_per_rev = 1000;
    p.wheel_diameter_m = 0.080f;
    p.track_width_m = 0.240f;
    return p;
}

static const float kPi = 3.14159265358979f;

TEST(ticks_to_metres_uses_wheel_circumference) {
    Odometry odo(params());
    CHECK_NEAR(odo.ticks_to_metres(1000), kPi * 0.080f, 1e-6);
    CHECK_NEAR(odo.ticks_to_metres(-500), -kPi * 0.040f, 1e-6);
}

TEST(straight_line_moves_x_only) {
    Odometry odo(params());
    odo.update(0, 0, 0.01f);
    odo.update(1000, 1000, 1.0f);  // one wheel turn each
    CHECK_NEAR(odo.pose().x, kPi * 0.080f, 1e-5);
    CHECK_NEAR(odo.pose().y, 0.0f, 1e-6);
    CHECK_NEAR(odo.pose().theta, 0.0f, 1e-6);
    CHECK_NEAR(odo.twist().v, kPi * 0.080f, 1e-5);
    CHECK_NEAR(odo.twist().w, 0.0f, 1e-6);
    CHECK_NEAR(odo.left_speed(), kPi * 0.080f, 1e-5);
}

TEST(spin_in_place_changes_theta_only) {
    Odometry odo(params());
    odo.update(0, 0, 0.01f);
    // Right wheel forward, left backward by the same amount: rotation only.
    // dtheta = (dr - dl) / track = 2 * d / 0.24
    const int32_t ticks = 100;
    odo.update(-ticks, ticks, 0.1f);
    const float d = kPi * 0.080f * ticks / 1000.0f;
    CHECK_NEAR(odo.pose().x, 0.0f, 1e-6);
    CHECK_NEAR(odo.pose().y, 0.0f, 1e-6);
    CHECK_NEAR(odo.pose().theta, 2.0f * d / 0.240f, 1e-5);
    CHECK_NEAR(odo.twist().v, 0.0f, 1e-6);
    CHECK_NEAR(odo.twist().w, 2.0f * d / 0.240f / 0.1f, 1e-4);
}

TEST(quarter_circle_arc_ends_at_expected_pose) {
    Odometry odo(params());
    odo.update(0, 0, 0.01f);
    // Drive an arc of radius R = 0.5 m turning left through 90 degrees in
    // many small steps. Left wheel radius R - t/2, right R + t/2.
    const float R = 0.5f, t = 0.240f;
    const float total = kPi / 2.0f;
    const int steps = 500;
    const float mpt = kPi * 0.080f / 1000.0f;
    float l_acc = 0.0f, r_acc = 0.0f;
    int32_t l = 0, r = 0;
    for (int i = 0; i < steps; ++i) {
        l_acc += (R - t / 2) * total / steps / mpt;
        r_acc += (R + t / 2) * total / steps / mpt;
        l = static_cast<int32_t>(std::lround(l_acc));
        r = static_cast<int32_t>(std::lround(r_acc));
        odo.update(l, r, 0.01f);
    }
    // After a 90 deg left turn of radius R the robot is at (R, R) heading pi/2.
    CHECK_NEAR(odo.pose().x, R, 0.01f);
    CHECK_NEAR(odo.pose().y, R, 0.01f);
    CHECK_NEAR(odo.pose().theta, kPi / 2.0f, 0.01f);
}

TEST(tick_counter_wraparound_is_handled) {
    Odometry odo(params());
    odo.update(2147483600, 2147483600, 0.01f);
    // 100 more ticks on each wheel wraps the int32 counters.
    const int32_t wrapped = static_cast<int32_t>(static_cast<uint32_t>(2147483600) + 100u);
    odo.update(wrapped, wrapped, 0.1f);
    CHECK_NEAR(odo.pose().x, kPi * 0.080f * 100.0f / 1000.0f, 1e-6);
}

TEST(theta_wraps_to_minus_pi_pi) {
    Odometry odo(params());
    odo.update(0, 0, 0.01f);
    for (int i = 0; i < 100; ++i) odo.update(-100 * (i + 1), 100 * (i + 1), 0.01f);
    CHECK(odo.pose().theta <= kPi && odo.pose().theta > -kPi);
}

TEST(reset_keeps_reference_ticks) {
    Odometry odo(params());
    odo.update(0, 0, 0.01f);
    odo.update(500, 500, 0.1f);
    odo.reset(500, 500);
    CHECK_NEAR(odo.pose().x, 0.0f, 1e-9);
    odo.update(600, 600, 0.1f);
    CHECK_NEAR(odo.pose().x, kPi * 0.080f * 0.1f, 1e-6);
}

TEST(inverse_kinematics) {
    float l, r;
    Odometry::twist_to_wheels(params(), 0.3f, 0.0f, l, r);
    CHECK_NEAR(l, 0.3f, 1e-6);
    CHECK_NEAR(r, 0.3f, 1e-6);
    Odometry::twist_to_wheels(params(), 0.0f, 1.0f, l, r);
    CHECK_NEAR(l, -0.12f, 1e-6);
    CHECK_NEAR(r, 0.12f, 1e-6);
}

MINITEST_MAIN("odometry")
