#!/usr/bin/env python3
"""Timed 90-degree turns plus LiDAR-based post-turn alignment."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

try:
    from .lidar_sectors import SectorMap
    from .wall_following import TwistCommand
except ImportError:  # pragma: no cover - direct script fallback
    from lidar_sectors import SectorMap
    from wall_following import TwistCommand


@dataclass(frozen=True)
class TurnStep:
    command: TwistCommand
    active: bool
    state: str
    reason: str


class TurnController:
    def __init__(
        self,
        turn_speed: float = 0.45,
        turn_degrees: float = 90.0,
        settle_seconds: float = 0.25,
        align_max_seconds: float = 1.6,
        align_yaw_limit: float = 0.28,
        align_gain: float = 0.45,
    ):
        self.turn_speed = abs(turn_speed)
        self.turn_seconds = abs(turn_degrees) / 57.2957795 / max(0.05, self.turn_speed)
        self.settle_seconds = settle_seconds
        self.align_max_seconds = align_max_seconds
        self.align_yaw_limit = align_yaw_limit
        self.align_gain = align_gain
        self.direction: Optional[str] = None
        self.phase = "IDLE"
        self.phase_started = 0.0
        self.completed_at = 0.0

    @property
    def active(self) -> bool:
        return self.phase != "IDLE"

    def start(self, direction: str, now: Optional[float] = None) -> bool:
        direction = direction.upper()
        if direction not in ("LEFT", "RIGHT"):
            return False
        if self.active:
            return False
        self.direction = direction
        self.phase = "TURNING"
        self.phase_started = now if now is not None else time.monotonic()
        return True

    def step(self, sectors: SectorMap, now: Optional[float] = None) -> TurnStep:
        now = now if now is not None else time.monotonic()
        if not self.active or self.direction is None:
            return TurnStep(TwistCommand(), False, "IDLE", "NO_ACTIVE_TURN")

        elapsed = now - self.phase_started
        sign = 1.0 if self.direction == "LEFT" else -1.0

        if self.phase == "TURNING":
            if elapsed < self.turn_seconds:
                return TurnStep(
                    TwistCommand(0.0, sign * self.turn_speed),
                    True,
                    f"TURNING_{self.direction}",
                    "TIMED_90_DEGREE_TURN",
                )
            self.phase = "SETTLING"
            self.phase_started = now
            return TurnStep(TwistCommand(), True, "SETTLING_AFTER_TURN", "STOP_BEFORE_ALIGNMENT")

        if self.phase == "SETTLING":
            if elapsed < self.settle_seconds:
                return TurnStep(TwistCommand(), True, "SETTLING_AFTER_TURN", "SETTLE")
            self.phase = "ALIGNING"
            self.phase_started = now
            elapsed = 0.0

        if self.phase == "ALIGNING":
            left = sectors.distance("left")
            right = sectors.distance("right")
            front = sectors.distance("front")
            if left is not None and right is not None:
                error = left - right
                yaw = max(-self.align_yaw_limit, min(self.align_yaw_limit, -self.align_gain * error))
                if abs(error) < 0.08 and (front is None or front > 0.45):
                    self._finish(now)
                    return TurnStep(TwistCommand(), False, "NAVIGATE", "ALIGNMENT_ERROR_SMALL")
                if elapsed < self.align_max_seconds:
                    return TurnStep(TwistCommand(0.0, yaw), True, "ALIGNING_AFTER_TURN", "CENTERING_BETWEEN_WALLS")

            if elapsed < self.align_max_seconds and front is not None and front > 0.50:
                return TurnStep(TwistCommand(0.02, 0.0), True, "ALIGNING_AFTER_TURN", "FRONT_CLEAR_NO_SIDE_PAIR")

            self._finish(now)
            return TurnStep(TwistCommand(), False, "NAVIGATE", "ALIGNMENT_TIMEOUT")

        self._finish(now)
        return TurnStep(TwistCommand(), False, "NAVIGATE", "TURN_STATE_RESET")

    def _finish(self, now: float) -> None:
        self.direction = None
        self.phase = "IDLE"
        self.phase_started = now
        self.completed_at = now

