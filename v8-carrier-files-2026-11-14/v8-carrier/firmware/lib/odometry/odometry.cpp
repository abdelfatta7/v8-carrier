#include "odometry.h"

#include <cmath>

namespace v8c {

namespace {
constexpr float kPi = 3.14159265358979f;
}

float wrap_angle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a <= -kPi) a += 2.0f * kPi;
    return a;
}

float Odometry::metres_per_tick() const {
    if (params_.ticks_per_rev <= 0) return 0.0f;
    return kPi * params_.wheel_diameter_m / static_cast<float>(params_.ticks_per_rev);
}

void Odometry::reset(int32_t left_ticks, int32_t right_ticks) {
    pose_ = Pose2D{};
    twist_ = Twist2D{};
    last_left_ = left_ticks;
    last_right_ = right_ticks;
    have_last_ = true;
    left_speed_ = right_speed_ = 0.0f;
}

void Odometry::update(int32_t left_ticks, int32_t right_ticks, float dt) {
    if (!have_last_) {
        reset(left_ticks, right_ticks);
        return;
    }
    // Unsigned subtraction then cast back gives the correct delta across an
    // int32 wrap-around.
    const int32_t d_left = static_cast<int32_t>(static_cast<uint32_t>(left_ticks) - static_cast<uint32_t>(last_left_));
    const int32_t d_right = static_cast<int32_t>(static_cast<uint32_t>(right_ticks) - static_cast<uint32_t>(last_right_));
    last_left_ = left_ticks;
    last_right_ = right_ticks;
    update_delta(d_left, d_right, dt);
}

void Odometry::update_delta(int32_t d_left, int32_t d_right, float dt) {
    const float dl = ticks_to_metres(d_left);
    const float dr = ticks_to_metres(d_right);
    const float d = 0.5f * (dl + dr);
    const float dtheta = (params_.track_width_m > 0.0f) ? (dr - dl) / params_.track_width_m : 0.0f;

    // Second-order (midpoint) integration: move along the average heading of
    // the step. Exact for straight lines and good to O(dtheta^2) for arcs.
    const float mid = pose_.theta + 0.5f * dtheta;
    pose_.x += d * std::cos(mid);
    pose_.y += d * std::sin(mid);
    pose_.theta = wrap_angle(pose_.theta + dtheta);

    if (dt > 0.0f) {
        left_speed_ = dl / dt;
        right_speed_ = dr / dt;
        twist_.v = d / dt;
        twist_.w = dtheta / dt;
    }
}

void Odometry::twist_to_wheels(const OdomParams& p, float v, float w, float& left, float& right) {
    left = v - 0.5f * w * p.track_width_m;
    right = v + 0.5f * w * p.track_width_m;
}

}  // namespace v8c
