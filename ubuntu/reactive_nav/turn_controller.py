#!/usr/bin/env python3
"""Timed 90-degree turns plus LiDAR-based post-turn alignment."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Dict, Optional

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
    debug: Dict[str, float | str | bool] = field(default_factory=dict)


class TurnController:
    def __init__(
        self,
        turn_speed: float = 0.45,
        turn_degrees: float = 90.0,
        settle_seconds: float = 0.25,
        align_max_seconds: float = 1.6,
        align_yaw_limit: float = 0.28,
        align_gain: float = 0.45,
        align_error_threshold: float = 0.08,
        align_stable_cycles: int = 3,
        align_same_direction_only: bool = True,
    ):
        self.turn_speed = abs(turn_speed)
        self.turn_degrees = abs(turn_degrees)
        self.turn_seconds = self.turn_degrees / 57.2957795 / max(0.05, self.turn_speed)
        self.active_turn_seconds = self.turn_seconds
        self.settle_seconds = settle_seconds
        self.align_max_seconds = align_max_seconds
        self.align_yaw_limit = align_yaw_limit
        self.align_gain = align_gain
        self.align_error_threshold = max(0.0, align_error_threshold)
        self.align_stable_cycles = max(1, int(align_stable_cycles))
        self.align_same_direction_only = bool(align_same_direction_only)
        self.direction: Optional[str] = None
        self.last_direction = "NONE"
        self.phase = "IDLE"
        self.phase_started = 0.0
        self.completed_at = 0.0
        self.started_at = 0.0
        self.align_stable_counter = 0
        self.last_align_error = 0.0
        self.last_align_yaw = 0.0
        self.last_align_yaw_clamped = False
        self.last_completed_reason = "NONE"

    @property
    def active(self) -> bool:
        return self.phase != "IDLE"

    def start(self, direction: str, now: Optional[float] = None) -> bool:
        direction = direction.upper()
        if direction not in ("LEFT", "RIGHT", "UTURN"):
            return False
        if self.active:
            return False
        self.direction = direction
        self.last_direction = direction
        turn_degrees = self.turn_degrees * 2.0 if direction == "UTURN" else self.turn_degrees
        self.active_turn_seconds = turn_degrees / 57.2957795 / max(0.05, self.turn_speed)
        self.phase = "TURNING"
        self.phase_started = now if now is not None else time.monotonic()
        self.started_at = self.phase_started
        self.align_stable_counter = 0
        self.last_align_error = 0.0
        self.last_align_yaw = 0.0
        self.last_align_yaw_clamped = False
        self.last_completed_reason = "ACTIVE"
        return True

    def step(self, sectors: SectorMap, now: Optional[float] = None) -> TurnStep:
        now = now if now is not None else time.monotonic()
        if not self.active or self.direction is None:
            return TurnStep(TwistCommand(), False, "IDLE", "NO_ACTIVE_TURN", self.snapshot())

        elapsed = now - self.phase_started
        sign = 1.0 if self.direction in ("LEFT", "UTURN") else -1.0

        if self.phase == "TURNING":
            if elapsed < self.active_turn_seconds:
                return TurnStep(
                    TwistCommand(0.0, sign * self.turn_speed),
                    True,
                    f"TURNING_{self.direction}",
                    "TIMED_90_DEGREE_TURN",
                    self.snapshot(turn_completed_reason="ACTIVE"),
                )
            self.phase = "SETTLING"
            self.phase_started = now
            return TurnStep(
                TwistCommand(),
                True,
                "SETTLING_AFTER_TURN",
                "STOP_BEFORE_ALIGNMENT",
                self.snapshot(turn_completed_reason="ACTIVE"),
            )

        if self.phase == "SETTLING":
            if elapsed < self.settle_seconds:
                return TurnStep(
                    TwistCommand(),
                    True,
                    "SETTLING_AFTER_TURN",
                    "SETTLE",
                    self.snapshot(turn_completed_reason="ACTIVE"),
                )
            self.phase = "ALIGNING"
            self.phase_started = now
            self.align_stable_counter = 0
            elapsed = 0.0

        if self.phase == "ALIGNING":
            left = sectors.distance("left")
            right = sectors.distance("right")
            if left is not None and right is not None:
                error = left - right
                yaw_raw = max(-self.align_yaw_limit, min(self.align_yaw_limit, -self.align_gain * error))
                yaw = yaw_raw
                yaw_clamped = False
                if self.align_same_direction_only:
                    if self.direction == "LEFT" and yaw < 0.0:
                        yaw = 0.0
                        yaw_clamped = True
                    if self.direction == "RIGHT" and yaw > 0.0:
                        yaw = 0.0
                        yaw_clamped = True
                self.last_align_error = error
                self.last_align_yaw = yaw
                self.last_align_yaw_clamped = yaw_clamped
                if abs(error) < self.align_error_threshold:
                    self.align_stable_counter += 1
                else:
                    self.align_stable_counter = 0
                if self.align_stable_counter >= self.align_stable_cycles:
                    self._finish(now, "ALIGNMENT_STABLE_SIDE_PAIR")
                    return TurnStep(
                        TwistCommand(),
                        False,
                        "NAVIGATE",
                        "ALIGNMENT_STABLE_SIDE_PAIR",
                        self.snapshot(),
                    )
                if elapsed < self.align_max_seconds:
                    return TurnStep(
                        TwistCommand(0.0, yaw),
                        True,
                        "ALIGNING_AFTER_TURN",
                        "CENTERING_BETWEEN_WALLS",
                        self.snapshot(turn_completed_reason="ACTIVE"),
                    )
            else:
                self.last_align_error = 0.0
                self.last_align_yaw = 0.0
                self.last_align_yaw_clamped = False
                self.align_stable_counter = 0
                if elapsed < self.align_max_seconds:
                    return TurnStep(
                        TwistCommand(0.0, 0.0),
                        True,
                        "ALIGNING_AFTER_TURN",
                        "FRONT_CLEAR_NO_SIDE_PAIR",
                        self.snapshot(turn_completed_reason="ACTIVE"),
                    )

            self._finish(now, "ALIGNMENT_TIMEOUT")
            return TurnStep(
                TwistCommand(),
                False,
                "NAVIGATE",
                "ALIGNMENT_TIMEOUT",
                self.snapshot(),
            )

        self._finish(now, "TURN_STATE_RESET")
        return TurnStep(TwistCommand(), False, "NAVIGATE", "TURN_STATE_RESET", self.snapshot())

    def snapshot(self, *, turn_completed_reason: Optional[str] = None) -> Dict[str, float | str | bool]:
        direction = self.direction or self.last_direction
        return {
            "turn_phase": self.phase,
            "turn_direction": direction.lower(),
            "turn_active": self.active,
            "turn_duration_s": max(0.0, time.monotonic() - self.started_at) if self.started_at > 0.0 else 0.0,
            "align_error": self.last_align_error,
            "align_yaw": self.last_align_yaw,
            "align_yaw_clamped": self.last_align_yaw_clamped,
            "align_stable_counter": float(self.align_stable_counter),
            "turn_completed_reason": turn_completed_reason or self.last_completed_reason,
        }

    def _finish(self, now: float, reason: str) -> None:
        self.last_completed_reason = reason
        self.direction = None
        self.phase = "IDLE"
        self.phase_started = now
        self.completed_at = now
