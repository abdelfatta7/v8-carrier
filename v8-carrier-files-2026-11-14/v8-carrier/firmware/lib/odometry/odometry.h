// Differential-drive odometry: encoder ticks -> wheel distance -> pose.
// Plain C++17, no Arduino includes. The same maths is mirrored in Python in
// ros2_ws/src/v8carrier_bringup/v8carrier_bringup/odometry.py.
#pragma once

#include <cstdint>

namespace v8c {

struct OdomParams {
    int32_t ticks_per_rev = 1320;     // encoder ticks per WHEEL revolution (after the gearbox, x4 quadrature)
    float wheel_diameter_m = 0.080f;  // cad/params.yaml wheels.diameter
    float track_width_m = 0.333f;     // cad/params.yaml wheels.track_width (derived 2026-11-10)
};

struct Pose2D {
    float x = 0.0f;      // m, forward of the start
    float y = 0.0f;      // m, left of the start
    float theta = 0.0f;  // rad, counter-clockwise, wrapped to (-pi, pi]
};

struct Twist2D {
    float v = 0.0f;  // m/s forward
    float w = 0.0f;  // rad/s counter-clockwise
};

class Odometry {
public:
    Odometry() = default;
    explicit Odometry(const OdomParams& p) : params_(p) {}

    void set_params(const OdomParams& p) { params_ = p; }
    const OdomParams& params() const { return params_; }

    float metres_per_tick() const;
    float ticks_to_metres(int32_t ticks) const { return static_cast<float>(ticks) * metres_per_tick(); }

    // Feed the cumulative tick counters (as the firmware counts them; they may
    // wrap around int32 and that is handled by taking the difference).
    // dt in seconds is used for the velocity estimate only.
    void update(int32_t left_ticks, int32_t right_ticks, float dt);

    // Convenience: feed tick deltas directly.
    void update_delta(int32_t d_left, int32_t d_right, float dt);

    const Pose2D& pose() const { return pose_; }
    const Twist2D& twist() const { return twist_; }

    // Restart from the origin while keeping the current tick counters as the
    // new reference, so the next update() does not produce a jump.
    void reset(int32_t left_ticks, int32_t right_ticks);

    // Per-wheel linear speed (m/s) from the last update; used by the wheel PIDs.
    float left_speed() const { return left_speed_; }
    float right_speed() const { return right_speed_; }

    // Wheel speed targets for a body twist (inverse kinematics). Static so the
    // command path can use it without an odometry instance.
    static void twist_to_wheels(const OdomParams& p, float v, float w, float& left, float& right);

private:
    OdomParams params_;
    Pose2D pose_;
    Twist2D twist_;
    int32_t last_left_ = 0;
    int32_t last_right_ = 0;
    bool have_last_ = false;
    float left_speed_ = 0.0f;
    float right_speed_ = 0.0f;
};

float wrap_angle(float a);

}  // namespace v8c
