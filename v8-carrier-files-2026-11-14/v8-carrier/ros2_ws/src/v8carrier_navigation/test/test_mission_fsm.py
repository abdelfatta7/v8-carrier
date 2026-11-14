import datetime as dt

from v8carrier_navigation.mission_fsm import MissionConfig, MissionFsm


def fsm():
    return MissionFsm(MissionConfig(vacuum_mode="soft", brush_duty=80, vacuum_start_timeout_s=20, vacuum_stop_timeout_s=20))


def test_full_mission_happy_path():
    m = fsm()
    assert m.state == "idle"
    acts = m.handle("start", 0.0)
    assert ("record_home",) in acts and ("vacuum", "soft") in acts and ("brush", 80) in acts
    assert m.state == "starting_vacuum"
    assert m.vacuum("off", "busy", 1.0) == []
    assert m.vacuum("soft", "idle", 3.0) == [("coverage", "start")]
    assert m.state == "covering"
    assert m.coverage("running", 4.0) == []
    acts = m.coverage("succeeded", 600.0)
    assert ("vacuum", "off") in acts and ("brush", 0) in acts
    assert m.state == "stopping_vacuum"
    assert m.vacuum("soft", "busy", 601.0) == []
    assert m.vacuum("off", "idle", 603.0) == [("nav_home",)]
    assert m.state == "returning"
    assert m.nav("succeeded", 650.0) == []
    assert m.state == "idle"
    assert m.detail == "home"


def test_stop_during_covering_turns_everything_off_and_stays_put():
    m = fsm()
    m.handle("start", 0.0)
    m.vacuum("soft", "idle", 2.0)
    acts = m.handle("stop", 10.0)
    assert ("coverage", "cancel") in acts and ("vacuum", "off") in acts and ("brush", 0) in acts
    assert m.state == "idle"
    # Vacuum confirmation after the stop does not trigger a return.
    assert m.vacuum("off", "idle", 12.0) == []
    assert m.state == "idle"


def test_go_home_during_covering_stops_vacuum_then_returns():
    m = fsm()
    m.handle("start", 0.0)
    m.vacuum("soft", "idle", 2.0)
    acts = m.handle("go_home", 100.0)
    assert ("coverage", "cancel") in acts and ("vacuum", "off") in acts
    assert m.state == "stopping_vacuum"
    assert m.vacuum("off", "idle", 103.0) == [("nav_home",)]
    assert m.state == "returning"
    assert m.handle("stop", 104.0) == [("brush", 0), ("nav_cancel",)]
    assert m.state == "idle"


def test_go_home_from_idle_needs_a_known_home():
    m = fsm()
    assert m.handle("go_home", 0.0) == []
    m.handle("start", 0.0)
    m.handle("stop", 1.0)
    assert m.handle("go_home", 2.0) == [("nav_home",)]
    assert m.state == "returning"


def test_vacuum_fault_while_starting_goes_to_fault_and_start_retries():
    m = fsm()
    m.handle("start", 0.0)
    assert m.vacuum("unknown", "fault", 5.0) == [("brush", 0)]
    assert m.state == "fault"
    assert m.handle("go_home", 6.0) == [("nav_home",)]
    m.nav("failed", 7.0)
    assert m.state == "idle"
    assert ("vacuum", "soft") in m.handle("start", 8.0)


def test_timeouts():
    m = fsm()
    m.handle("start", 0.0)
    assert m.tick(19.0) == []
    assert m.tick(21.0) == [("brush", 0)]
    assert m.state == "fault"

    m = fsm()
    m.handle("start", 0.0)
    m.vacuum("soft", "idle", 1.0)
    m.coverage("succeeded", 100.0)
    assert m.tick(121.0) == [("nav_home",)]  # stop unconfirmed, still go home
    assert m.state == "returning"
    assert "unconfirmed" in m.detail


def test_coverage_failure_still_stops_vacuum_and_returns():
    m = fsm()
    m.handle("start", 0.0)
    m.vacuum("soft", "idle", 1.0)
    assert ("vacuum", "off") in m.coverage("failed", 50.0)
    assert m.vacuum("off", "idle", 52.0) == [("nav_home",)]


def test_schedule_starts_once_per_day():
    m = fsm()
    assert m.set_schedule("09:30")
    assert not m.set_schedule("25:00")
    day1 = dt.datetime(2026, 11, 10, 9, 30, 5)
    assert m.tick(0.0, dt.datetime(2026, 11, 10, 9, 29, 59)) == []
    acts = m.tick(1.0, day1)
    assert ("vacuum", "soft") in acts
    assert m.state == "starting_vacuum"
    m.handle("stop", 2.0)
    assert m.tick(3.0, dt.datetime(2026, 11, 10, 9, 30, 50)) == []  # same day: not again
    assert ("vacuum", "soft") in m.tick(4.0, dt.datetime(2026, 11, 11, 9, 30, 0))
    m.handle("stop", 5.0)
    assert m.set_schedule("")
    assert m.schedule is None
    assert m.tick(6.0, dt.datetime(2026, 11, 12, 9, 30, 0)) == []
    assert m.status()["schedule"] == ""


def test_status_dict():
    m = fsm()
    s = m.status()
    assert s["state"] == "idle" and s["home_known"] is False
