// Dyson V8 power-button control. The V8 here has ONE push button that cycles
// soft -> hard -> off (decided 2026-11-10). A hobby servo taps it and the
// microphone confirms the result. This module is the state machine only: it
// decides the servo angle and interprets the mic; main.cpp moves the servo
// and feeds the mic state.
//
// Press routine for each tap: press, release, wait for the mic to settle,
// confirm, on mismatch retry once, then fault (mode becomes UNKNOWN).
// A request for a mode taps the right number of times. From UNKNOWN the first
// tap is used to find out where we are.
//
// Plain C++17, no Arduino includes.
#pragma once

#include <cstdint>

namespace v8c {

enum class VacMode : uint8_t { Off = 0, Soft = 1, Hard = 2, Unknown = 3 };
enum class VacStatus : uint8_t { Idle = 0, Busy = 1, Fault = 2 };

struct VacuumButtonConfig {
    uint8_t rest_angle_deg = 20;        // servo clear of the button
    uint8_t press_angle_deg = 70;       // servo pushing the button through its travel
    uint32_t press_ms = 300;            // held down
    uint32_t release_ms = 200;          // after lifting, before the mic is consulted
    uint32_t confirm_ms = 1500;         // spin-up / spin-down plus the detector's 0.5 s hold
    uint32_t between_presses_ms = 400;  // gap between taps of a multi-tap request
    uint8_t max_retries = 1;            // "retry once"
    uint16_t hard_level = 0;            // > 0: mic level above this means HARD, below SOFT
    uint32_t self_stop_ms = 3000;       // running expected but quiet this long -> stopped by itself
};

// The next mode after one tap of the button.
VacMode next_mode(VacMode m);
// Number of taps to get from `from` to `to` (0..2); from Unknown returns 1.
uint8_t presses_needed(VacMode from, VacMode to);

class VacuumButton {
public:
    VacuumButton() = default;
    explicit VacuumButton(const VacuumButtonConfig& c) : cfg_(c) {}

    void set_config(const VacuumButtonConfig& c) { cfg_ = c; }
    const VacuumButtonConfig& config() const { return cfg_; }

    // Ask for a mode. Returns false (and does nothing) while busy or faulted.
    bool request(VacMode target);

    // Call every tick (e.g. 100 Hz) with the mic detector's state and level.
    void tick(uint32_t now_ms, bool mic_running, uint16_t mic_level);

    uint8_t servo_angle() const { return servo_angle_; }
    VacMode mode() const { return mode_; }
    VacStatus status() const { return status_; }
    bool busy() const { return status_ == VacStatus::Busy; }

    // Clear a fault so new requests are accepted again.
    void clear_fault();
    // Tell the module what the vacuum is doing (e.g. "off" at boot by the owner).
    void set_mode_hint(VacMode m) { if (!busy()) mode_ = m; }

    // True exactly once after the vacuum stopped on its own (battery flat or
    // bin full) while we believed it to be running.
    bool take_self_stop_event();
    // Taps performed since boot (diagnostics).
    uint32_t press_count() const { return press_count_; }

private:
    enum class Step : uint8_t { Idle, Press, Release, Confirm, Gap };

    void start_press(uint32_t now_ms);
    void evaluate(uint32_t now_ms, bool mic_running, uint16_t mic_level);
    void finish(VacStatus s);
    VacMode mode_from_mic(bool mic_running, uint16_t mic_level) const;

    VacuumButtonConfig cfg_;
    VacMode mode_ = VacMode::Unknown;
    VacStatus status_ = VacStatus::Idle;
    Step step_ = Step::Idle;
    uint32_t step_since_ = 0;
    uint8_t servo_angle_ = 20;

    VacMode target_ = VacMode::Unknown;
    uint8_t presses_left_ = 0;
    uint8_t retries_left_ = 0;
    VacMode mode_before_press_ = VacMode::Unknown;
    uint32_t press_count_ = 0;

    bool quiet_timer_running_ = false;
    uint32_t quiet_since_ = 0;
    bool self_stop_event_ = false;
    bool loud_timer_running_ = false;
    uint32_t loud_since_ = 0;
};

}  // namespace v8c
