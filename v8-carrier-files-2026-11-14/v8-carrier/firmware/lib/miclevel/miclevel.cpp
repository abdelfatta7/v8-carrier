#include "miclevel.h"

#include <cmath>

namespace v8c {

bool RmsWindow::add(int16_t sample) {
    // Remove the DC offset with a very slow first-order filter.
    dc_ += (static_cast<float>(sample) - dc_) * (1.0f / 4096.0f);
    const float centred = static_cast<float>(sample) - dc_;
    const int64_t c = static_cast<int64_t>(centred);
    sum_sq_ += static_cast<uint64_t>(c * c);
    if (++count_ >= window_) {
        const double mean_sq = static_cast<double>(sum_sq_) / static_cast<double>(count_);
        double rms = std::sqrt(mean_sq);
        if (rms > 65535.0) rms = 65535.0;
        level_ = static_cast<uint16_t>(rms + 0.5);
        count_ = 0;
        sum_sq_ = 0;
        ++completed_;
        return true;
    }
    return false;
}

size_t RmsWindow::add_block(const int16_t* samples, size_t n) {
    size_t windows = 0;
    for (size_t i = 0; i < n; ++i)
        if (add(samples[i])) ++windows;
    return windows;
}

void RunningDetector::set_config(const MicDetectorConfig& c) {
    cfg_ = c;
    floor_ = c.initial_floor;
}

uint16_t RunningDetector::on_threshold() const {
    float t = floor_ * cfg_.on_ratio;
    if (t < cfg_.min_on_level) t = cfg_.min_on_level;
    if (t > 65535.0f) t = 65535.0f;
    return static_cast<uint16_t>(t);
}

uint16_t RunningDetector::off_threshold() const {
    return static_cast<uint16_t>(on_threshold() * cfg_.off_ratio);
}

bool RunningDetector::update(uint32_t now_ms, uint16_t level) {
    // Comparator with hysteresis.
    const bool want_running = running_ ? (level > off_threshold()) : (level > on_threshold());

    if (want_running == running_) {
        pending_ = false;
    } else {
        if (!pending_) {
            pending_ = true;
            pending_since_ = now_ms;
        } else if (now_ms - pending_since_ >= cfg_.hold_ms) {
            running_ = want_running;
            pending_ = false;
        }
    }

    // Track the noise floor only while the vacuum is (believed to be) off,
    // otherwise the vacuum's own noise would become the floor.
    if (!running_ && !pending_) floor_ += (static_cast<float>(level) - floor_) * cfg_.floor_alpha;

    return running_;
}

}  // namespace v8c
