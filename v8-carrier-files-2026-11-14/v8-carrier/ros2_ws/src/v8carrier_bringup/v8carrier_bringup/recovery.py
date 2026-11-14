"""Estop recovery on the Pi side. Pure Python.

The firmware latches the estop by itself (docs/protocol.md). This decides what
the serial bridge sends back:

  * latched by the heartbeat only (the Pi was away): CLEAR_ESTOP as soon as the
    cause has cleared, because the Pi being back is the fix;
  * latched by bumper and/or cliff only: back away at `backoff_speed` for
    `backoff_s`, then CLEAR_ESTOP once the causes are gone (the firmware
    accepts reverse commands in this state); up to `max_attempts` per latch,
    then wait for the operator;
  * low battery in the causes: do nothing, the operator (or the charger) must
    sort it out.

`update()` takes the time and the latest Status and returns a list of actions:
("wheels", left_mm_s, right_mm_s), ("clear",), or nothing. The bridge keeps
sending the operator's cmd_vel when the list is empty.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from .protocol import (ESTOP_BUMPER, ESTOP_CLIFF, ESTOP_HEARTBEAT, ESTOP_LOW_BATTERY, Status)

BACKOFF_CAUSES = ESTOP_BUMPER | ESTOP_CLIFF


@dataclass
class RecoveryConfig:
    auto_clear_heartbeat: bool = True
    auto_backoff: bool = True
    backoff_speed_mm_s: int = 60
    backoff_s: float = 1.0
    settle_s: float = 0.3          # after backing off, wait for the causes to clear
    max_attempts: int = 3
    retry_wait_s: float = 5.0      # between attempts


class EstopRecovery:
    IDLE = "idle"
    BACKING = "backing"
    SETTLING = "settling"
    WAITING = "waiting"       # exhausted attempts or a cause we do not handle
    DONE = "done"             # clear sent, waiting to see the latch drop

    def __init__(self, config: Optional[RecoveryConfig] = None) -> None:
        self.cfg = config or RecoveryConfig()
        self.state = self.IDLE
        self.attempts = 0
        self._since = 0.0
        self._was_latched = False

    @property
    def overriding_wheels(self) -> bool:
        return self.state == self.BACKING

    def update(self, now: float, status: Status) -> List[Tuple]:
        actions: List[Tuple] = []
        latched = status.estop_latched

        if not latched:
            if self._was_latched or self.state != self.IDLE:
                self.state = self.IDLE
                self.attempts = 0
            self._was_latched = False
            return actions
        self._was_latched = True

        causes = status.estop_latch_cause
        active = status.estop_flags & ~0x01

        if causes & ESTOP_LOW_BATTERY:
            self.state = self.WAITING
            return actions

        if self.state in (self.IDLE, self.DONE, self.WAITING):
            if self.state == self.WAITING and now - self._since < self.cfg.retry_wait_s:
                return actions
            if self.state == self.DONE and now - self._since < self.cfg.settle_s:
                return actions  # give the firmware time to drop the latch
            if active == 0:
                # Nothing active any more: whichever cause it was, clearing is safe
                # (heartbeat-only needs no motion; a bumper that released needs none either).
                if causes & ~ESTOP_HEARTBEAT == 0 and not self.cfg.auto_clear_heartbeat:
                    self.state = self.WAITING
                    self._since = now
                    return actions
                actions.append(("clear",))
                self.state = self.DONE
                self._since = now
                return actions
            if active & ~BACKOFF_CAUSES:
                # Heartbeat still lost (bridge just reconnected): wait a tick.
                return actions
            # Bumper / cliff pressed: back away.
            if not self.cfg.auto_backoff or self.attempts >= self.cfg.max_attempts:
                if self.state != self.WAITING:
                    self.state = self.WAITING
                    self._since = now
                return actions
            self.attempts += 1
            self.state = self.BACKING
            self._since = now

        if self.state == self.BACKING:
            if now - self._since < self.cfg.backoff_s:
                actions.append(("wheels", -self.cfg.backoff_speed_mm_s, -self.cfg.backoff_speed_mm_s))
            else:
                actions.append(("wheels", 0, 0))
                self.state = self.SETTLING
                self._since = now
            return actions

        if self.state == self.SETTLING:
            if now - self._since >= self.cfg.settle_s:
                if active == 0:
                    actions.append(("clear",))
                    self.state = self.DONE
                    self._since = now
                else:
                    self.state = self.WAITING if self.attempts >= self.cfg.max_attempts else self.IDLE
                    self._since = now
            return actions

        return actions
