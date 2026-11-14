#include "../minitest.h"
#include "safety.h"

using namespace v8c;

static SafetyInputs healthy() {
    SafetyInputs in;
    in.bumper_bits = 0;
    for (int k = 0; k < 4; ++k) in.cliff_mv[k] = 2000;
    in.pack_v = 16.0f;
    return in;
}

// Runs the monitor with a healthy robot and a fresh heartbeat so that the
// only causes are the ones the test injects.
static SafetyMonitor alive_monitor(uint32_t now) {
    SafetyMonitor m;
    m.heartbeat(now);
    m.update(now, healthy());
    CHECK(!m.latched());
    return m;
}

TEST(latched_at_boot_until_heartbeat_and_clear) {
    SafetyMonitor m;
    CHECK(m.update(0, healthy()));
    CHECK(m.latched());
    CHECK_EQ(m.active_causes(), CAUSE_HEARTBEAT);
    CHECK_EQ(m.latch_cause(), CAUSE_HEARTBEAT);
    CHECK(!m.clear());  // cause still active
    m.heartbeat(10);
    m.update(10, healthy());
    CHECK(m.latched());  // stays latched...
    CHECK_EQ(m.active_causes(), 0);
    CHECK(m.clear());  // ...until the explicit clear
    CHECK(!m.latched());
    CHECK_EQ(m.flags(), 0);
}

TEST(heartbeat_timeout_500ms) {
    SafetyMonitor m = alive_monitor(1000);
    m.update(1400, healthy());
    CHECK(!m.latched());
    m.update(1500, healthy());
    CHECK(!m.latched());  // exactly 500 ms is still fine
    m.update(1501, healthy());
    CHECK(m.latched());
    CHECK_EQ(m.latch_cause(), CAUSE_HEARTBEAT);
    CHECK(!m.motion_allowed(-0.1f, -0.1f));  // no back-off exception for a lost Pi
}

TEST(bumper_is_debounced_and_latches) {
    SafetyMonitor m = alive_monitor(0);
    SafetyInputs in = healthy();
    in.bumper_bits = 0x01;
    m.update(10, in);
    CHECK(!m.latched());  // one sample is not enough (debounce 2)
    m.update(20, in);
    CHECK(m.latched());
    CHECK_EQ(m.flags(), CAUSE_LATCHED | CAUSE_BUMPER);
    // Bumper still pressed: reversing is allowed, anything else is not.
    CHECK(m.motion_allowed(-0.1f, -0.1f));
    CHECK(m.motion_allowed(0.0f, -0.1f));
    CHECK(!m.motion_allowed(0.1f, -0.1f));
    CHECK(!m.motion_allowed(0.1f, 0.1f));
    CHECK(!m.clear());
    // Released: cause clears, latch stays until the command.
    m.update(30, healthy());
    CHECK(m.latched());
    CHECK_EQ(m.active_causes(), 0);
    CHECK(m.clear());
    CHECK(!m.latched());
    CHECK(m.motion_allowed(0.3f, 0.3f));
}

TEST(cliff_no_floor_latches_after_debounce) {
    SafetyMonitor m = alive_monitor(0);
    SafetyInputs in = healthy();
    in.cliff_mv[2] = 400;  // ahead of the left wheel: no floor
    m.update(10, in);
    m.update(20, in);
    CHECK(!m.latched());
    m.update(30, in);
    CHECK(m.latched());
    CHECK_EQ(m.latch_cause(), CAUSE_CLIFF);
    CHECK(m.motion_allowed(-0.05f, -0.05f));
    // A glitch of two samples does not trip it.
    SafetyMonitor g = alive_monitor(0);
    g.update(10, in);
    g.update(20, in);
    g.update(30, healthy());
    g.update(40, in);
    g.update(50, in);
    CHECK(!g.latched());
}

TEST(disconnected_cliff_sensor_reads_zero_and_is_unsafe) {
    SafetyMonitor m = alive_monitor(0);
    SafetyInputs in = healthy();
    in.cliff_mv[0] = 0;
    for (int i = 1; i <= 3; ++i) m.update(10 * i, in);
    CHECK(m.latched());
}

// Runs the monitor from t0 to t1 in 100 ms steps with the Pi alive.
static void run_alive(SafetyMonitor& m, uint32_t t0, uint32_t t1, const SafetyInputs& in) {
    for (uint32_t t = t0; t <= t1; t += 100) {
        m.heartbeat(t);
        m.update(t, in);
    }
}

TEST(low_battery_needs_hold_and_has_hysteresis) {
    SafetyMonitor m = alive_monitor(0);
    SafetyInputs in = healthy();
    in.pack_v = 13.5f;
    run_alive(m, 100, 1500, in);
    CHECK(!m.latched());  // 1.4 s below: a motor-start sag could do that
    run_alive(m, 1600, 2200, in);
    CHECK(m.latched());
    CHECK_EQ(m.latch_cause(), CAUSE_LOW_BATTERY);
    CHECK(!m.motion_allowed(-0.1f, -0.1f));
    // Recovering to 14.2 V is not enough (clear level 14.6).
    in.pack_v = 14.2f;
    run_alive(m, 2300, 2500, in);
    CHECK(!m.clear());
    in.pack_v = 14.8f;
    run_alive(m, 2600, 2800, in);
    CHECK(m.clear());
}

TEST(sag_shorter_than_hold_does_not_trip) {
    SafetyMonitor m = alive_monitor(0);
    SafetyInputs low = healthy();
    low.pack_v = 13.0f;
    run_alive(m, 100, 1000, low);
    run_alive(m, 1100, 3000, healthy());  // recovered after 1 s
    run_alive(m, 3100, 4500, low);        // timer restarts: 1.4 s is below the 2 s hold
    CHECK(!m.latched());
    run_alive(m, 4600, 5200, low);
    CHECK(m.latched());
}

TEST(latch_cause_accumulates_until_cleared) {
    SafetyMonitor m = alive_monitor(0);
    SafetyInputs in = healthy();
    in.bumper_bits = 2;
    m.update(10, in);
    m.update(20, in);
    CHECK_EQ(m.latch_cause(), CAUSE_BUMPER);
    // Now the Pi goes quiet as well.
    m.update(600, in);
    CHECK_EQ(m.latch_cause(), CAUSE_BUMPER | CAUSE_HEARTBEAT);
    CHECK(!m.motion_allowed(-0.1f, -0.1f));  // heartbeat cause forbids back-off
    m.heartbeat(700);
    m.update(700, healthy());
    CHECK(m.clear());
    CHECK_EQ(m.latch_cause(), 0);
}

MINITEST_MAIN("safety")
