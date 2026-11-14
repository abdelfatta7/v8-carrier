#include "../minitest.h"
#include "vacuum_button.h"

using namespace v8c;

// A pretend Dyson: one button cycling off -> soft -> hard -> off, and a mic
// that hears it (soft 4000, hard 7000, off 100). The press registers on the
// servo's rest -> press edge, unless the sim is told to drop presses.
struct SimVacuum {
    VacMode truth = VacMode::Off;
    int drop_next_presses = 0;  // number of upcoming presses that do not register
    int presses_seen = 0;
    bool last_pressed = false;

    bool running() const { return truth != VacMode::Off; }
    uint16_t level() const {
        switch (truth) {
            case VacMode::Soft: return 4000;
            case VacMode::Hard: return 7000;
            default: return 100;
        }
    }
    void observe_servo(uint8_t angle, uint8_t press_angle) {
        const bool pressed = angle == press_angle;
        if (pressed && !last_pressed) {
            ++presses_seen;
            if (drop_next_presses > 0)
                --drop_next_presses;
            else
                truth = next_mode(truth);
        }
        last_pressed = pressed;
    }
};

static VacuumButtonConfig cfg(uint16_t hard_level = 0) {
    VacuumButtonConfig c;
    c.rest_angle_deg = 20;
    c.press_angle_deg = 70;
    c.press_ms = 300;
    c.release_ms = 200;
    c.confirm_ms = 1500;
    c.between_presses_ms = 400;
    c.max_retries = 1;
    c.hard_level = hard_level;
    c.self_stop_ms = 3000;
    return c;
}

// Runs the button against the sim from t0 for `duration` ms in 10 ms ticks.
static uint32_t run(VacuumButton& b, SimVacuum& sim, uint32_t t0, uint32_t duration) {
    uint32_t t = t0;
    for (; t < t0 + duration; t += 10) {
        b.tick(t, sim.running(), sim.level());
        sim.observe_servo(b.servo_angle(), 70);
    }
    return t;
}

TEST(presses_needed_table) {
    CHECK_EQ(presses_needed(VacMode::Off, VacMode::Soft), 1);
    CHECK_EQ(presses_needed(VacMode::Off, VacMode::Hard), 2);
    CHECK_EQ(presses_needed(VacMode::Soft, VacMode::Off), 2);
    CHECK_EQ(presses_needed(VacMode::Hard, VacMode::Off), 1);
    CHECK_EQ(presses_needed(VacMode::Soft, VacMode::Soft), 0);
    CHECK_EQ(presses_needed(VacMode::Unknown, VacMode::Off), 1);
    CHECK(next_mode(VacMode::Hard) == VacMode::Off);
    CHECK(next_mode(VacMode::Unknown) == VacMode::Unknown);
}

TEST(single_press_off_to_soft_confirmed_by_mic) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Off);
    SimVacuum sim;
    CHECK(b.request(VacMode::Soft));
    CHECK(b.busy());
    CHECK(!b.request(VacMode::Hard));  // busy
    run(b, sim, 0, 3000);
    CHECK_EQ(sim.presses_seen, 1);
    CHECK(!b.busy());
    CHECK_EQ(static_cast<int>(b.status()), static_cast<int>(VacStatus::Idle));
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Soft));
    CHECK_EQ(b.servo_angle(), 20);
}

TEST(two_presses_off_to_hard) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Off);
    SimVacuum sim;
    CHECK(b.request(VacMode::Hard));
    run(b, sim, 0, 6000);
    CHECK_EQ(sim.presses_seen, 2);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Hard));
    CHECK_EQ(static_cast<int>(sim.truth), static_cast<int>(VacMode::Hard));
    CHECK(!b.busy());
}

TEST(hard_to_off_is_one_press_soft_to_off_is_two) {
    VacuumButton b(cfg());
    SimVacuum sim;
    sim.truth = VacMode::Hard;
    b.set_mode_hint(VacMode::Hard);
    CHECK(b.request(VacMode::Off));
    run(b, sim, 0, 3000);
    CHECK_EQ(sim.presses_seen, 1);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Off));

    VacuumButton b2(cfg());
    SimVacuum sim2;
    sim2.truth = VacMode::Soft;
    b2.set_mode_hint(VacMode::Soft);
    CHECK(b2.request(VacMode::Off));
    run(b2, sim2, 0, 6000);
    CHECK_EQ(sim2.presses_seen, 2);
    CHECK_EQ(static_cast<int>(b2.mode()), static_cast<int>(VacMode::Off));
    CHECK_EQ(static_cast<int>(sim2.truth), static_cast<int>(VacMode::Off));
}

TEST(request_for_current_mode_does_nothing) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Soft);
    SimVacuum sim;
    sim.truth = VacMode::Soft;
    CHECK(b.request(VacMode::Soft));
    CHECK(!b.busy());
    run(b, sim, 0, 1000);
    CHECK_EQ(sim.presses_seen, 0);
}

TEST(missed_press_is_retried_once_then_succeeds) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Off);
    SimVacuum sim;
    sim.drop_next_presses = 1;
    CHECK(b.request(VacMode::Soft));
    run(b, sim, 0, 6000);
    CHECK_EQ(sim.presses_seen, 2);
    CHECK_EQ(static_cast<int>(b.status()), static_cast<int>(VacStatus::Idle));
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Soft));
}

TEST(two_missed_presses_fault_and_mode_becomes_unknown) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Off);
    SimVacuum sim;
    sim.drop_next_presses = 2;
    CHECK(b.request(VacMode::Soft));
    // Two full press cycles (300 + 200 + 1500 ms each, 400 ms gap) end at 4.4 s.
    run(b, sim, 0, 5000);
    CHECK_EQ(sim.presses_seen, 2);
    CHECK_EQ(static_cast<int>(b.status()), static_cast<int>(VacStatus::Fault));
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Unknown));
    CHECK(!b.request(VacMode::Soft));  // faulted: refused
    // Still quiet after 3 s: the mode settles to OFF while the fault stays.
    run(b, sim, 5000, 3500);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Off));
    CHECK_EQ(static_cast<int>(b.status()), static_cast<int>(VacStatus::Fault));
    b.clear_fault();
    CHECK(b.request(VacMode::Soft));
}

TEST(unknown_mode_is_discovered_by_first_press) {
    // The vacuum is really HARD but we do not know; asking for OFF should
    // press once, hear silence, conclude OFF and stop.
    VacuumButton b(cfg());
    SimVacuum sim;
    sim.truth = VacMode::Hard;
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Unknown));
    CHECK(b.request(VacMode::Off));
    run(b, sim, 0, 3000);
    CHECK_EQ(sim.presses_seen, 1);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Off));
    CHECK(!b.busy());

    // Really OFF and unknown; asking for HARD: discovery press -> SOFT, then one more.
    VacuumButton b2(cfg());
    SimVacuum sim2;
    CHECK(b2.request(VacMode::Hard));
    run(b2, sim2, 0, 6000);
    CHECK_EQ(sim2.presses_seen, 2);
    CHECK_EQ(static_cast<int>(b2.mode()), static_cast<int>(VacMode::Hard));
    CHECK_EQ(static_cast<int>(sim2.truth), static_cast<int>(VacMode::Hard));
}

TEST(calibrated_hard_level_distinguishes_soft_from_hard) {
    VacuumButton b(cfg(6000));
    SimVacuum sim;
    sim.truth = VacMode::Hard;  // unknown to the button
    CHECK(b.request(VacMode::Soft));
    // Discovery press: hard -> off (quiet) => OFF; then one press -> SOFT.
    run(b, sim, 0, 6000);
    CHECK_EQ(sim.presses_seen, 2);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Soft));
    CHECK_EQ(static_cast<int>(sim.truth), static_cast<int>(VacMode::Soft));
}

TEST(vacuum_stopping_by_itself_raises_event) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Soft);
    SimVacuum sim;
    sim.truth = VacMode::Soft;
    run(b, sim, 0, 1000);
    CHECK(!b.take_self_stop_event());
    sim.truth = VacMode::Off;  // battery flat
    run(b, sim, 1000, 2000);
    CHECK(!b.take_self_stop_event());  // not yet 3 s
    run(b, sim, 3000, 1500);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Off));
    CHECK(b.take_self_stop_event());
    CHECK(!b.take_self_stop_event());  // one-shot
}

TEST(unknown_and_quiet_settles_to_off_without_event) {
    VacuumButton b(cfg());
    SimVacuum sim;
    run(b, sim, 0, 4000);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Off));
    CHECK(!b.take_self_stop_event());
}

TEST(hand_press_while_off_makes_mode_unknown) {
    VacuumButton b(cfg());
    b.set_mode_hint(VacMode::Off);
    SimVacuum sim;
    sim.truth = VacMode::Soft;  // the owner pressed it by hand
    run(b, sim, 0, 4000);
    CHECK_EQ(static_cast<int>(b.mode()), static_cast<int>(VacMode::Unknown));
    // With a calibrated hard level we can tell which.
    VacuumButton b2(cfg(6000));
    b2.set_mode_hint(VacMode::Off);
    run(b2, sim, 0, 4000);
    CHECK_EQ(static_cast<int>(b2.mode()), static_cast<int>(VacMode::Soft));
}

MINITEST_MAIN("vacuum_button")
