#include <cmath>
#include <vector>

#include "../minitest.h"
#include "miclevel.h"

using namespace v8c;

TEST(rms_of_square_wave_is_amplitude) {
    RmsWindow w(100);
    int windows = 0;
    for (int i = 0; i < 100; ++i) windows += w.add((i % 2) ? 1000 : -1000) ? 1 : 0;
    CHECK_EQ(windows, 1);
    CHECK_NEAR(w.level(), 1000, 2);  // DC filter barely moves for a symmetric signal
    CHECK_EQ(w.windows_completed(), 1u);
}

TEST(rms_of_sine_is_amplitude_over_sqrt2) {
    RmsWindow w(1000);
    std::vector<int16_t> buf;
    for (int i = 0; i < 1000; ++i) buf.push_back(static_cast<int16_t>(8000.0 * std::sin(2 * 3.14159265 * i / 50.0)));
    CHECK_EQ(w.add_block(buf.data(), buf.size()), 1u);
    CHECK_NEAR(w.level(), 8000.0 / std::sqrt(2.0), 60);
}

TEST(rms_removes_dc_offset_eventually) {
    RmsWindow w(512);
    // Constant offset with a tiny signal: after many windows the DC estimate
    // converges and the level drops towards the signal's own RMS.
    uint16_t first = 0, last = 0;
    for (int n = 0; n < 60; ++n) {
        for (int i = 0; i < 512; ++i) w.add(static_cast<int16_t>(3000 + ((i % 2) ? 50 : -50)));
        if (n == 0) first = w.level();
        last = w.level();
    }
    CHECK(first > 2000);
    CHECK(last < 200);
}

TEST(silence_gives_zero) {
    RmsWindow w(16);
    for (int i = 0; i < 16; ++i) w.add(0);
    CHECK_EQ(w.level(), 0);
}

static MicDetectorConfig cfg() {
    MicDetectorConfig c;
    c.min_on_level = 1500;
    c.on_ratio = 4.0f;
    c.off_ratio = 0.6f;
    c.hold_ms = 500;
    c.floor_alpha = 0.1f;
    c.initial_floor = 100;
    return c;
}

TEST(detector_needs_500ms_hold_before_turning_on) {
    RunningDetector d(cfg());
    CHECK(!d.running());
    CHECK_EQ(d.on_threshold(), 1500);
    // Loud, but not yet for 500 ms.
    CHECK(!d.update(0, 5000));
    CHECK(!d.update(200, 5000));
    CHECK(!d.update(499, 5000));
    CHECK(d.update(500, 5000));
    CHECK(d.running());
}

TEST(detector_ignores_short_bursts) {
    RunningDetector d(cfg());
    d.update(0, 5000);
    d.update(300, 5000);
    d.update(400, 50);  // a door slam ends
    d.update(900, 50);
    CHECK(!d.running());
    // A short dip while running does not switch it off either.
    for (uint32_t t = 1000; t <= 1600; t += 100) d.update(t, 5000);
    CHECK(d.running());
    d.update(1700, 100);
    d.update(2000, 100);
    CHECK(d.running());
    d.update(2100, 5000);
    d.update(2700, 5000);
    CHECK(d.running());
}

TEST(detector_hysteresis_between_on_and_off) {
    RunningDetector d(cfg());
    for (uint32_t t = 0; t <= 600; t += 100) d.update(t, 5000);
    CHECK(d.running());
    const uint16_t off = d.off_threshold();
    CHECK_EQ(off, 900);  // 1500 * 0.6
    // Level between the off and on thresholds keeps it running.
    for (uint32_t t = 700; t <= 2000; t += 100) d.update(t, 1200);
    CHECK(d.running());
    // Below the off threshold for 500 ms switches it off.
    for (uint32_t t = 2100; t <= 2700; t += 100) d.update(t, 800);
    CHECK(!d.running());
}

TEST(noise_floor_adapts_only_while_off_and_raises_threshold) {
    RunningDetector d(cfg());
    for (uint32_t t = 0; t < 100000; t += 100) d.update(t, 600);  // a noisy room
    CHECK_NEAR(d.noise_floor(), 600.0f, 1.0f);
    CHECK_NEAR(d.on_threshold(), 2400, 2);  // 600 * 4 > the 1500 minimum
    // Now the vacuum runs; the floor must not follow it.
    for (uint32_t t = 100000; t < 110000; t += 100) d.update(t, 6000);
    CHECK(d.running());
    CHECK_NEAR(d.noise_floor(), 600.0f, 1.0f);
}

MINITEST_MAIN("miclevel")
