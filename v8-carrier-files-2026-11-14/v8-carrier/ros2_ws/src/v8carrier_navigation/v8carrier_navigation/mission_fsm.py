"""Mission state machine: start the vacuum, cover, stop the vacuum, return to
the start. Pure Python; the mission node feeds it events and carries out the
actions it returns.

States  idle, starting_vacuum, covering, stopping_vacuum, returning, fault
Events  handle("start"|"stop"|"go_home")
        vacuum(mode, status)      from /vacuum/state ("off"/"soft"/"hard"/"unknown", "idle"/"busy"/"fault")
        coverage(status)          "running"/"succeeded"/"failed"/"canceled"
        nav(status)               "succeeded"/"failed"/"canceled"
        tick(now)                 timeouts and the daily schedule
Actions ("record_home",)  ("vacuum", mode)  ("brush", duty)  ("coverage", "start"|"cancel")
        ("nav_home",)  ("nav_cancel",)
"""

from __future__ import annotations

import datetime as dt
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Action = Tuple


@dataclass
class MissionConfig:
    vacuum_mode: str = "soft"
    brush_duty: int = 80
    vacuum_start_timeout_s: float = 20.0
    vacuum_stop_timeout_s: float = 20.0


@dataclass
class MissionFsm:
    cfg: MissionConfig = field(default_factory=MissionConfig)
    state: str = "idle"
    detail: str = ""
    since: float = 0.0
    schedule: Optional[str] = None      # "HH:MM" local time, daily
    home_known: bool = False
    _stop_reason: str = ""
    _last_scheduled_day: Optional[dt.date] = None

    # ---- helpers --------------------------------------------------------------
    def _go(self, state: str, now: float, detail: str = "") -> None:
        self.state = state
        self.detail = detail
        self.since = now

    def status(self) -> dict:
        return {"state": self.state, "detail": self.detail, "schedule": self.schedule or "",
                "since": self.since, "home_known": self.home_known}

    # ---- commands -------------------------------------------------------------
    def handle(self, command: str, now: float) -> List[Action]:
        if command == "start":
            if self.state in ("idle", "fault"):
                self._go("starting_vacuum", now, f"waiting for the vacuum to run {self.cfg.vacuum_mode}")
                self.home_known = True
                return [("record_home",), ("brush", self.cfg.brush_duty), ("vacuum", self.cfg.vacuum_mode)]
            return []
        if command == "stop":
            actions: List[Action] = [("brush", 0)]
            if self.state == "covering":
                actions.append(("coverage", "cancel"))
            if self.state == "returning":
                actions.append(("nav_cancel",))
            if self.state in ("starting_vacuum", "covering", "stopping_vacuum"):
                actions.append(("vacuum", "off"))
            self._stop_reason = "stop"
            self._go("idle", now, "stopped by command")
            return actions
        if command == "go_home":
            if self.state == "covering":
                self._stop_reason = "go_home"
                self._go("stopping_vacuum", now, "going home: waiting for the vacuum to stop")
                return [("coverage", "cancel"), ("vacuum", "off"), ("brush", 0)]
            if self.state == "starting_vacuum":
                self._stop_reason = "go_home"
                self._go("stopping_vacuum", now, "going home: waiting for the vacuum to stop")
                return [("vacuum", "off"), ("brush", 0)]
            if self.state in ("idle", "fault") and self.home_known:
                self._go("returning", now, "going home")
                return [("nav_home",)]
            return []
        return []

    # ---- feedback -------------------------------------------------------------
    def vacuum(self, mode: str, status: str, now: float) -> List[Action]:
        if self.state == "starting_vacuum":
            if status == "fault":
                self._go("fault", now, "vacuum button fault: press not confirmed")
                return [("brush", 0)]
            if status == "idle" and mode == self.cfg.vacuum_mode:
                self._go("covering", now, "covering")
                return [("coverage", "start")]
        elif self.state == "stopping_vacuum":
            if status == "idle" and mode == "off" or status == "fault":
                return self._after_vacuum_stopped(now, "" if status == "idle" else "vacuum fault; ")
        return []

    def _after_vacuum_stopped(self, now: float, prefix: str = "") -> List[Action]:
        if self._stop_reason == "stop" or not self.home_known:
            self._go("idle", now, prefix + "done")
            return []
        self._go("returning", now, prefix + "returning to the start")
        return [("nav_home",)]

    def coverage(self, status: str, now: float) -> List[Action]:
        if self.state != "covering":
            return []
        if status == "succeeded":
            self._stop_reason = "done"
            self._go("stopping_vacuum", now, "coverage done: waiting for the vacuum to stop")
            return [("vacuum", "off"), ("brush", 0)]
        if status in ("failed", "canceled"):
            self._stop_reason = status
            self._go("stopping_vacuum", now, f"coverage {status}: waiting for the vacuum to stop")
            return [("vacuum", "off"), ("brush", 0)]
        return []

    def nav(self, status: str, now: float) -> List[Action]:
        if self.state != "returning":
            return []
        self._go("idle", now, "home" if status == "succeeded" else f"return {status}")
        return []

    # ---- time -----------------------------------------------------------------
    def tick(self, now: float, local_time: Optional[dt.datetime] = None) -> List[Action]:
        actions: List[Action] = []
        if self.state == "starting_vacuum" and now - self.since > self.cfg.vacuum_start_timeout_s:
            self._go("fault", now, "vacuum did not start in time")
            actions.append(("brush", 0))
        elif self.state == "stopping_vacuum" and now - self.since > self.cfg.vacuum_stop_timeout_s:
            actions += self._after_vacuum_stopped(now, "vacuum stop unconfirmed; ")
        if local_time is not None and self.schedule and self.state == "idle":
            if (local_time.strftime("%H:%M") == self.schedule
                    and self._last_scheduled_day != local_time.date()):
                self._last_scheduled_day = local_time.date()
                actions += self.handle("start", now)
        return actions

    def set_schedule(self, hhmm: str) -> bool:
        hhmm = (hhmm or "").strip()
        if not hhmm:
            self.schedule = None
            return True
        try:
            dt.datetime.strptime(hhmm, "%H:%M")
        except ValueError:
            return False
        self.schedule = hhmm
        self._last_scheduled_day = None
        return True
