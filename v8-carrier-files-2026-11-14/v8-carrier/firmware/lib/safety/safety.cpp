#include "safety.h"

namespace v8c {

void SafetyMonitor::heartbeat(uint32_t now_ms) {
    hb_seen_ = true;
    hb_last_ms_ = now_ms;
}

bool SafetyMonitor::update(uint32_t now_ms, const SafetyInputs& in) {
    uint8_t active = 0;

    // Bumpers: debounced count of consecutive pressed samples.
    if (in.bumper_bits != 0) {
        if (bumper_count_ < 255) ++bumper_count_;
    } else {
        bumper_count_ = 0;
    }
    if (bumper_count_ >= cfg_.bumper_debounce) active |= CAUSE_BUMPER;

    // Cliff sensors: each must see the floor; a low reading for several
    // consecutive samples means a drop (or a dead sensor, which is also unsafe).
    for (int k = 0; k < 4; ++k) {
        if (in.cliff_mv[k] < cfg_.cliff_floor_min_mv) {
            if (cliff_count_[k] < 255) ++cliff_count_[k];
        } else {
            cliff_count_[k] = 0;
        }
        if (cliff_count_[k] >= cfg_.cliff_debounce) active |= CAUSE_CLIFF;
    }

    // Low battery: must stay below the threshold for the hold time (motor
    // start-up sag is short); clears with hysteresis above the clear level.
    if (!low_batt_) {
        if (in.pack_v < cfg_.low_voltage_v) {
            if (!low_timer_running_) {
                low_timer_running_ = true;
                low_since_ms_ = now_ms;
            } else if (now_ms - low_since_ms_ >= cfg_.low_voltage_hold_ms) {
                low_batt_ = true;
            }
        } else {
            low_timer_running_ = false;
        }
    } else if (in.pack_v > cfg_.low_voltage_clear_v) {
        low_batt_ = false;
        low_timer_running_ = false;
    }
    if (low_batt_) active |= CAUSE_LOW_BATTERY;

    // Heartbeat: lost if never seen or older than the timeout.
    if (!hb_seen_ || (now_ms - hb_last_ms_) > cfg_.heartbeat_timeout_ms) active |= CAUSE_HEARTBEAT;

    active_ = active;
    if (active != 0 && !latched_) {
        latched_ = true;
        latch_cause_ = active;
    } else if (active != 0 && latched_) {
        // Keep accumulating causes that appear while latched, so the Pi can
        // see everything that happened before it clears.
        latch_cause_ = static_cast<uint8_t>(latch_cause_ | active);
    }
    return latched_;
}

bool SafetyMonitor::clear() {
    if (active_ != 0) return false;
    latched_ = false;
    latch_cause_ = 0;
    return true;
}

bool SafetyMonitor::motion_allowed(float left_vel, float right_vel) const {
    if (!latched_) return true;
    constexpr uint8_t kBackOffCauses = CAUSE_BUMPER | CAUSE_CLIFF;
    const bool only_back_off_causes =
        (latch_cause_ & ~kBackOffCauses) == 0 && (active_ & ~kBackOffCauses) == 0;
    const bool reversing = left_vel <= 0.0f && right_vel <= 0.0f;
    return only_back_off_causes && reversing;
}

}  // namespace v8c
