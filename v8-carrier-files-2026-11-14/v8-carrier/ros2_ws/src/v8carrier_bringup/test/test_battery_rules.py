from v8carrier_bringup.battery_rules import BatteryRules, BatteryRulesConfig


def cfg():
    return BatteryRulesConfig(low_voltage_v=14.4, low_hold_s=10.0, low_clear_v=14.8)


def test_low_voltage_needs_hold_and_mission():
    r = BatteryRules(cfg())
    assert r.update(0, 14.0, "soft", "covering") is None
    assert r.update(5, 14.0, "soft", "covering") is None
    assert r.update(9.9, 14.0, "soft", "covering") is None
    assert r.update(10.0, 14.0, "soft", "covering") == "go_home"
    assert "below" in r.last_reason
    # Only once per mission.
    assert r.update(11, 14.0, "soft", "covering") is None
    assert r.update(20, 13.9, "soft", "returning") is None


def test_low_voltage_while_idle_does_not_send():
    r = BatteryRules(cfg())
    for t in range(0, 60, 5):
        assert r.update(t, 13.0, "off", "idle") is None


def test_short_sag_resets_timer():
    r = BatteryRules(cfg())
    r.update(0, 14.0, "soft", "covering")
    r.update(5, 15.0, "soft", "covering")   # recovered above clear level
    assert r.update(12, 14.0, "soft", "covering") is None  # timer restarted at 12
    assert r.update(21, 14.0, "soft", "covering") is None
    assert r.update(22, 14.0, "soft", "covering") == "go_home"


def test_hysteresis_between_low_and_clear():
    r = BatteryRules(cfg())
    r.update(0, 14.0, "soft", "covering")
    r.update(5, 14.6, "soft", "covering")   # between low and clear: timer keeps running
    assert r.update(10, 14.3, "soft", "covering") == "go_home"


def test_vacuum_self_stop_sends_home_once():
    r = BatteryRules(cfg())
    assert r.update(0, 16.0, "off", "starting_vacuum") is None
    assert r.update(1, 16.0, "soft", "covering") is None
    assert r.update(2, 16.0, "off", "covering") == "go_home"
    assert r.last_reason == "vacuum stopped by itself"
    assert r.update(3, 16.0, "off", "covering") is None
    assert r.update(4, 16.0, "soft", "covering") is None
    assert r.update(5, 16.0, "off", "covering") is None  # already sent this mission


def test_commanded_stop_is_not_a_self_stop():
    r = BatteryRules(cfg())
    r.update(0, 16.0, "soft", "covering")
    assert r.update(1, 16.0, "soft", "stopping_vacuum") is None
    assert r.update(2, 16.0, "off", "stopping_vacuum") is None
    assert r.update(3, 16.0, "off", "returning") is None


def test_new_mission_rearms_rules():
    r = BatteryRules(cfg())
    r.update(0, 16.0, "soft", "covering")
    assert r.update(1, 16.0, "off", "covering") == "go_home"
    r.update(2, 16.0, "off", "returning")
    r.update(3, 16.0, "off", "idle")
    r.update(4, 16.0, "soft", "covering")  # new mission
    assert r.update(5, 16.0, "off", "covering") == "go_home"


def test_missing_readings_are_tolerated():
    r = BatteryRules(cfg())
    assert r.update(0, None, None, None) is None
    assert r.update(1, None, "soft", "covering") is None
