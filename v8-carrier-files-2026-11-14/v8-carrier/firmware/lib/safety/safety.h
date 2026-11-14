// Emergency stop that needs no Pi. The motors are cut (latched) when any
// bumper closes, any cliff sensor reads "no floor", the pack voltage stays
// below a threshold, or the Pi heartbeat is older than the timeout. The latch
// clears only by an explicit command, and only after every cause has cleared.
//
// One deliberate exception: while latched by a bumper or a cliff only, slow
// reverse motion (both wheels <= 0) is allowed so the robot can back away from
// the wall or the edge, otherwise the bumper stays pressed forever and the
// latch can never clear. The latch itself stays set until CLEAR_ESTOP.
//
// Plain C++17, no Arduino includes.
#pragma once

#include <cstdint>

namespace v8c {

struct SafetyConfig {
    float low_voltage_v = 14.0f;         // 4S at 3.5 V/cell; below this (held) -> estop
    float low_voltage_clear_v = 14.6f;   // hysteresis: the cause clears above this
    uint32_t low_voltage_hold_ms = 2000; // must stay low this long (motor sag is shorter)
    uint16_t cliff_floor_min_mv = 900;   // Sharp GP2Y0A51SK0F: floor at 3-4 cm reads ~1.8-2.2 V,
                                         // a drop reads < 0.6 V, a disconnected sensor reads 0
    uint8_t cliff_debounce = 3;          // consecutive "no floor" samples before it counts
    uint8_t bumper_debounce = 2;         // consecutive pressed samples before it counts
    uint32_t heartbeat_timeout_ms = 500; // Pi heartbeat older than this -> estop
};

struct SafetyInputs {
    uint8_t bumper_bits = 0;              // bit0 left, bit1 right, 1 = pressed
    uint16_t cliff_mv[4] = {0, 0, 0, 0};  // analog cliff sensor readings
    float pack_v = 0.0f;                  // pack voltage
};

// Cause bits; the same values as proto::EstopBits so the status message can
// carry them directly without a translation table.
enum SafetyCause : uint8_t {
    CAUSE_LATCHED = 1u << 0,
    CAUSE_BUMPER = 1u << 1,
    CAUSE_CLIFF = 1u << 2,
    CAUSE_LOW_BATTERY = 1u << 3,
    CAUSE_HEARTBEAT = 1u << 4,
};

class SafetyMonitor {
public:
    SafetyMonitor() = default;
    explicit SafetyMonitor(const SafetyConfig& c) : cfg_(c) {}

    void set_config(const SafetyConfig& c) { cfg_ = c; }
    const SafetyConfig& config() const { return cfg_; }

    // Call when a HEARTBEAT (or SET_WHEEL_VEL) frame arrives from the Pi.
    void heartbeat(uint32_t now_ms);

    // Call every control tick with fresh sensor readings. Latches when a
    // cause becomes active. Returns true when the latch is set.
    bool update(uint32_t now_ms, const SafetyInputs& in);

    bool latched() const { return latched_; }
    uint8_t active_causes() const { return active_; }   // CAUSE_* bits (never CAUSE_LATCHED)
    uint8_t latch_cause() const { return latch_cause_; } // causes active when the latch set
    uint8_t flags() const { return static_cast<uint8_t>(active_ | (latched_ ? CAUSE_LATCHED : 0)); }

    // Explicit clear from the Pi. Succeeds only when no cause is active.
    bool clear();

    // Whether this wheel command may be applied right now. Anything is allowed
    // when not latched; while latched only reverse motion is allowed, and only
    // when the latch and the active causes are bumper/cliff alone.
    bool motion_allowed(float left_vel, float right_vel) const;

    // True once at least one heartbeat has ever arrived (used for logging).
    bool heartbeat_seen() const { return hb_seen_; }

private:
    SafetyConfig cfg_;
    bool latched_ = false;
    uint8_t active_ = 0;
    uint8_t latch_cause_ = 0;

    bool hb_seen_ = false;
    uint32_t hb_last_ms_ = 0;

    uint8_t bumper_count_ = 0;
    uint8_t cliff_count_[4] = {0, 0, 0, 0};

    bool low_batt_ = false;        // the (hysteretic) low-battery cause
    bool low_timer_running_ = false;
    uint32_t low_since_ms_ = 0;
};

}  // namespace v8c
