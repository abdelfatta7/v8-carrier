from v8carrier_bringup import protocol as P
from v8carrier_bringup.recovery import EstopRecovery, RecoveryConfig


def status(latched=False, active=0, latch_cause=0):
    flags = (P.ESTOP_LATCHED if latched else 0) | active
    return P.Status(estop_flags=flags, estop_latch_cause=latch_cause)


def test_not_latched_does_nothing():
    r = EstopRecovery()
    assert r.update(0.0, status()) == []
    assert r.state == EstopRecovery.IDLE


def test_heartbeat_latch_is_cleared_once_pi_is_back():
    r = EstopRecovery()
    # Pi just reconnected: the cause is still active for a moment.
    assert r.update(0.0, status(True, P.ESTOP_HEARTBEAT, P.ESTOP_HEARTBEAT)) == []
    assert r.update(0.1, status(True, 0, P.ESTOP_HEARTBEAT)) == [("clear",)]
    assert r.state == EstopRecovery.DONE
    # Not repeated every tick while the firmware processes it.
    assert r.update(0.2, status(True, 0, P.ESTOP_HEARTBEAT)) == []
    assert r.update(0.3, status(False)) == []
    assert r.state == EstopRecovery.IDLE


def test_heartbeat_auto_clear_can_be_disabled():
    r = EstopRecovery(RecoveryConfig(auto_clear_heartbeat=False))
    assert r.update(0.0, status(True, 0, P.ESTOP_HEARTBEAT)) == []
    assert r.state == EstopRecovery.WAITING


def test_bumper_backoff_then_clear():
    r = EstopRecovery(RecoveryConfig(backoff_speed_mm_s=60, backoff_s=1.0, settle_s=0.3))
    pressed = status(True, P.ESTOP_BUMPER, P.ESTOP_BUMPER)
    assert r.update(0.0, pressed) == [("wheels", -60, -60)]
    assert r.overriding_wheels
    assert r.update(0.5, pressed) == [("wheels", -60, -60)]
    # Bumper releases while backing; keep backing until the time is up.
    released = status(True, 0, P.ESTOP_BUMPER)
    assert r.update(0.9, released) == [("wheels", -60, -60)]
    assert r.update(1.0, released) == [("wheels", 0, 0)]
    assert r.state == EstopRecovery.SETTLING
    assert r.update(1.1, released) == []
    assert r.update(1.3, released) == [("clear",)]
    assert r.update(1.4, status(False)) == []
    assert r.state == EstopRecovery.IDLE
    assert r.attempts == 0


def test_bumper_still_pressed_after_backoff_retries_then_waits():
    r = EstopRecovery(RecoveryConfig(backoff_s=1.0, settle_s=0.3, max_attempts=2, retry_wait_s=5.0))
    pressed = status(True, P.ESTOP_BUMPER, P.ESTOP_BUMPER)
    t = 0.0
    attempts_started = 0
    for _ in range(400):
        acts = r.update(t, pressed)
        if acts == [("wheels", -60, -60)] and r.state == EstopRecovery.BACKING and r.attempts > attempts_started:
            attempts_started = r.attempts
        t += 0.1
    assert attempts_started == 2
    assert r.state == EstopRecovery.WAITING
    # A later release still gets a clear.
    assert ("clear",) in r.update(t + 10, status(True, 0, P.ESTOP_BUMPER))


def test_low_battery_is_left_to_the_operator():
    r = EstopRecovery()
    assert r.update(0.0, status(True, P.ESTOP_LOW_BATTERY, P.ESTOP_LOW_BATTERY)) == []
    assert r.update(1.0, status(True, 0, P.ESTOP_LOW_BATTERY)) == []
    assert r.state == EstopRecovery.WAITING


def test_bumper_released_without_motion_is_just_cleared():
    r = EstopRecovery()
    assert r.update(0.0, status(True, 0, P.ESTOP_BUMPER)) == [("clear",)]


def test_cliff_with_heartbeat_lost_waits_for_pi_first():
    r = EstopRecovery()
    both = status(True, P.ESTOP_CLIFF | P.ESTOP_HEARTBEAT, P.ESTOP_CLIFF | P.ESTOP_HEARTBEAT)
    assert r.update(0.0, both) == []
    assert r.state == EstopRecovery.IDLE
    cliff_only = status(True, P.ESTOP_CLIFF, P.ESTOP_CLIFF | P.ESTOP_HEARTBEAT)
    assert r.update(0.2, cliff_only) == [("wheels", -60, -60)]
