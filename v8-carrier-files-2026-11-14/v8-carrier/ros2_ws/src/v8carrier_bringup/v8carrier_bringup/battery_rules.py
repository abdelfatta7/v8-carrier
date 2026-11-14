"""Battery rules: when to send the robot home. Pure Python.

Two rules, both only while a mission is covering:
  1. the pack voltage has been below `low_voltage_v` for `low_hold_s`
     (loaded voltage sags; a short dip is not a flat pack);
  2. the vacuum went from running (soft/hard) to off without the mission
     asking for it: its own battery is flat or its bin is full.

Each rule emits "go_home" once per mission; `reset()` is called when a new
mission starts. `update()` returns the command to publish, or None.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

COVERING_STATES = ("covering",)
RUNNING_MODES = ("soft", "hard")


@dataclass
class BatteryRulesConfig:
    low_voltage_v: float = 14.4    # 3.6 V/cell: above the firmware's 14.0 V hard cut
    low_hold_s: float = 10.0
    low_clear_v: float = 14.8      # hysteresis: the low timer resets above this


class BatteryRules:
    def __init__(self, config: Optional[BatteryRulesConfig] = None) -> None:
        self.cfg = config or BatteryRulesConfig()
        self.reset()

    def reset(self) -> None:
        self._low_since: Optional[float] = None
        self._sent_low = False
        self._sent_vacuum = False
        self._last_vacuum_mode: Optional[str] = None
        self._last_mission_state: Optional[str] = None
        self.last_reason: Optional[str] = None

    def update(self, now: float, pack_v: Optional[float], vacuum_mode: Optional[str],
               mission_state: Optional[str]) -> Optional[str]:
        """Feed the latest readings; returns "go_home" or None."""
        # A new mission resets the once-per-mission flags.
        if mission_state != self._last_mission_state:
            if mission_state in COVERING_STATES and self._last_mission_state not in COVERING_STATES:
                self._sent_low = False
                self._sent_vacuum = False
            self._last_mission_state = mission_state

        covering = mission_state in COVERING_STATES
        command = None

        # Rule 1: pack voltage.
        if pack_v is not None:
            if pack_v < self.cfg.low_voltage_v:
                if self._low_since is None:
                    self._low_since = now
                elif covering and not self._sent_low and now - self._low_since >= self.cfg.low_hold_s:
                    self._sent_low = True
                    command = "go_home"
                    self.last_reason = f"pack {pack_v:.2f} V below {self.cfg.low_voltage_v:.2f} V for {self.cfg.low_hold_s:.0f} s"
            elif pack_v > self.cfg.low_clear_v:
                self._low_since = None

        # Rule 2: the vacuum stopped by itself.
        if vacuum_mode is not None:
            if (covering and self._last_vacuum_mode in RUNNING_MODES and vacuum_mode == "off"
                    and not self._sent_vacuum):
                self._sent_vacuum = True
                if command is None:
                    command = "go_home"
                    self.last_reason = "vacuum stopped by itself"
            self._last_vacuum_mode = vacuum_mode

        return command
