// Microphone level: RMS over a sample window, then a "vacuum running" detector
// with a running (slowly adapting) noise-floor threshold, hysteresis and a
// 0.5 s hold, as the build manual specifies. Plain C++17, no Arduino includes.
//
// Levels are in 16-bit sample units (the INMP441's 24-bit samples are shifted
// right by 8 in main.cpp before they get here), so a quiet room is a few tens
// to a few hundreds and a running vacuum at close range several thousands.
// The default thresholds are placeholders until cad/measurements.yaml
// sound_levels is filled in.
#pragma once

#include <cstddef>
#include <cstdint>

namespace v8c {

class RmsWindow {
public:
    explicit RmsWindow(size_t window_samples = 1024) : window_(window_samples ? window_samples : 1) {}

    // Feed one sample. Returns true when a window has just completed and
    // level() holds a new value.
    bool add(int16_t sample);
    // Feed a block; returns the number of windows completed.
    size_t add_block(const int16_t* samples, size_t n);

    uint16_t level() const { return level_; }    // RMS of the last completed window
    size_t window() const { return window_; }
    float dc_offset() const { return dc_; }
    uint32_t windows_completed() const { return completed_; }

private:
    size_t window_;
    size_t count_ = 0;
    uint64_t sum_sq_ = 0;
    float dc_ = 0.0f;        // slow DC estimate removed from every sample
    uint16_t level_ = 0;
    uint32_t completed_ = 0;
};

struct MicDetectorConfig {
    uint16_t min_on_level = 1500;  // absolute floor for the "on" threshold
    float on_ratio = 4.0f;         // on threshold = max(min_on_level, noise_floor * on_ratio)
    float off_ratio = 0.6f;        // off threshold = on threshold * off_ratio (hysteresis)
    uint32_t hold_ms = 500;        // the comparator must agree for this long before the state flips
    float floor_alpha = 0.02f;     // noise-floor EMA weight per update, applied only while not running
    uint16_t initial_floor = 100;  // starting noise floor estimate
};

class RunningDetector {
public:
    RunningDetector() = default;
    explicit RunningDetector(const MicDetectorConfig& c) : cfg_(c), floor_(c.initial_floor) {}

    void set_config(const MicDetectorConfig& c);
    const MicDetectorConfig& config() const { return cfg_; }

    // Feed a new level (one per completed RMS window) with the current time.
    // Returns the running state after this update.
    bool update(uint32_t now_ms, uint16_t level);

    bool running() const { return running_; }
    float noise_floor() const { return floor_; }
    uint16_t on_threshold() const;
    uint16_t off_threshold() const;

private:
    MicDetectorConfig cfg_;
    float floor_ = 100.0f;
    bool running_ = false;
    bool pending_ = false;        // the comparator currently disagrees with running_
    uint32_t pending_since_ = 0;
};

}  // namespace v8c
