#include "vacuum_button.h"

namespace v8c {

VacMode next_mode(VacMode m) {
    switch (m) {
        case VacMode::Off: return VacMode::Soft;
        case VacMode::Soft: return VacMode::Hard;
        case VacMode::Hard: return VacMode::Off;
        default: return VacMode::Unknown;
    }
}

uint8_t presses_needed(VacMode from, VacMode to) {
    if (from == VacMode::Unknown || to == VacMode::Unknown) return 1;
    const int a = static_cast<int>(from), b = static_cast<int>(to);
    return static_cast<uint8_t>(((b - a) % 3 + 3) % 3);
}

bool VacuumButton::request(VacMode target) {
    if (status_ != VacStatus::Idle || target == VacMode::Unknown) return false;
    target_ = target;
    presses_left_ = presses_needed(mode_, target);
    if (presses_left_ == 0) return true;  // already there
    retries_left_ = cfg_.max_retries;
    status_ = VacStatus::Busy;
    step_ = Step::Gap;  // the first tick starts the press
    step_since_ = 0;
    quiet_timer_running_ = loud_timer_running_ = false;
    return true;
}

void VacuumButton::start_press(uint32_t now_ms) {
    mode_before_press_ = mode_;
    servo_angle_ = cfg_.press_angle_deg;
    step_ = Step::Press;
    step_since_ = now_ms;
    ++press_count_;
}

void VacuumButton::finish(VacStatus s) {
    status_ = s;
    step_ = Step::Idle;
    servo_angle_ = cfg_.rest_angle_deg;
    presses_left_ = 0;
}

void VacuumButton::clear_fault() {
    if (status_ == VacStatus::Fault) status_ = VacStatus::Idle;
}

bool VacuumButton::take_self_stop_event() {
    const bool e = self_stop_event_;
    self_stop_event_ = false;
    return e;
}

VacMode VacuumButton::mode_from_mic(bool mic_running, uint16_t mic_level) const {
    if (!mic_running) return VacMode::Off;
    if (cfg_.hard_level > 0) return (mic_level >= cfg_.hard_level) ? VacMode::Hard : VacMode::Soft;
    return VacMode::Soft;  // cannot tell soft from hard without a calibrated level: assume soft
}

void VacuumButton::evaluate(uint32_t now_ms, bool mic_running, uint16_t mic_level) {
    const VacMode heard = mode_from_mic(mic_running, mic_level);
    bool ok;
    VacMode new_mode;

    if (mode_before_press_ == VacMode::Unknown) {
        // Discovery tap: whatever the mic says is where we are now.
        ok = true;
        new_mode = heard;
    } else {
        const VacMode expected = next_mode(mode_before_press_);
        if (cfg_.hard_level == 0) {
            // Only "running or not" is observable: soft and hard both run.
            const bool expect_running = expected != VacMode::Off;
            ok = (expect_running == mic_running);
        } else {
            ok = (heard == expected);
        }
        new_mode = expected;
    }

    if (ok) {
        mode_ = new_mode;
        // Recompute from the (possibly newly learned) mode.
        presses_left_ = presses_needed(mode_, target_);
        if (presses_left_ == 0) {
            finish(VacStatus::Idle);
        } else {
            retries_left_ = cfg_.max_retries;
            step_ = Step::Gap;
            step_since_ = now_ms;
        }
    } else if (retries_left_ > 0) {
        // Assume the tap did not register and tap again from the same state.
        --retries_left_;
        step_ = Step::Gap;
        step_since_ = now_ms;
    } else {
        mode_ = VacMode::Unknown;
        finish(VacStatus::Fault);
    }
}

void VacuumButton::tick(uint32_t now_ms, bool mic_running, uint16_t mic_level) {
    switch (step_) {
        case Step::Idle:
            break;
        case Step::Gap:
            if (step_since_ == 0 || now_ms - step_since_ >= cfg_.between_presses_ms) start_press(now_ms);
            return;
        case Step::Press:
            if (now_ms - step_since_ >= cfg_.press_ms) {
                servo_angle_ = cfg_.rest_angle_deg;
                step_ = Step::Release;
                step_since_ = now_ms;
            }
            return;
        case Step::Release:
            if (now_ms - step_since_ >= cfg_.release_ms) {
                step_ = Step::Confirm;
                step_since_ = now_ms;
            }
            return;
        case Step::Confirm:
            if (now_ms - step_since_ >= cfg_.confirm_ms) evaluate(now_ms, mic_running, mic_level);
            return;
    }

    // Idle: watch for the vacuum changing state on its own.
    if (mode_ == VacMode::Soft || mode_ == VacMode::Hard || mode_ == VacMode::Unknown) {
        if (!mic_running) {
            if (!quiet_timer_running_) {
                quiet_timer_running_ = true;
                quiet_since_ = now_ms;
            } else if (now_ms - quiet_since_ >= cfg_.self_stop_ms) {
                if (mode_ != VacMode::Unknown) self_stop_event_ = true;
                mode_ = VacMode::Off;
                quiet_timer_running_ = false;
            }
        } else {
            quiet_timer_running_ = false;
        }
    }
    if (mode_ == VacMode::Off) {
        // Running while we believe it is off: someone pressed the button by hand.
        if (mic_running) {
            if (!loud_timer_running_) {
                loud_timer_running_ = true;
                loud_since_ = now_ms;
            } else if (now_ms - loud_since_ >= cfg_.self_stop_ms) {
                mode_ = mode_from_mic(true, mic_level);
                if (cfg_.hard_level == 0) mode_ = VacMode::Unknown;  // cannot tell which
                loud_timer_running_ = false;
            }
        } else {
            loud_timer_running_ = false;
        }
    }
}

}  // namespace v8c
