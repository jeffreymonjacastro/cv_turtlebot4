#!/usr/bin/env python3
"""Replaceable navigation modules for TurtleBot4 reactive navigation."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Dict, Protocol

try:
    from .lidar_sectors import SectorMap, largest_free_gap
except ImportError:  # pragma: no cover - direct script fallback
    from lidar_sectors import SectorMap, largest_free_gap


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class NavigationSuggestion:
    command: TwistCommand
    mode: str
    reason: str
    debug: Dict[str, float | str] = field(default_factory=dict)


@dataclass(frozen=True)
class NavigationObservation:
    sectors: SectorMap
    now: float
    dt: float


class NavigationModule(Protocol):
    name: str

    def compute(self, observation: NavigationObservation) -> NavigationSuggestion:
        ...


class WallFollowNavigation:
    """Slow corridor/wall following with largest-free-sector recovery."""

    name = "wall_follow"

    def __init__(
        self,
        base_speed: float = 0.10,
        narrow_speed: float = 0.06,
        max_yaw: float = 0.65,
        kp: float = 0.45,
        kd: float = 0.04,
        desired_wall_distance: float = 0.42,
        front_clear_distance: float = 0.55,
        recovery_clearance: float = 0.42,
        side_avoid_distance: float = 0.34,
        front_corner_avoid_distance: float = 0.62,
        avoidance_gain: float = 0.65,
    ):
        self.base_speed = base_speed
        self.narrow_speed = narrow_speed
        self.max_yaw = max_yaw
        self.kp = kp
        self.kd = kd
        self.desired_wall_distance = desired_wall_distance
        self.front_clear_distance = front_clear_distance
        self.recovery_clearance = recovery_clearance
        self.side_avoid_distance = side_avoid_distance
        self.front_corner_avoid_distance = front_corner_avoid_distance
        self.avoidance_gain = avoidance_gain
        self._last_error = None
        self._last_time = time.monotonic()

    def compute(self, observation: NavigationObservation) -> NavigationSuggestion:
        sectors = observation.sectors
        left = sectors.distance("left")
        right = sectors.distance("right")
        front = sectors.distance("front")
        front_left = sectors.distance("front_left")
        front_right = sectors.distance("front_right")

        if sectors.valid_count == 0:
            return NavigationSuggestion(TwistCommand(), "IDLE", "NO_VALID_LIDAR_POINTS")

        if front is None:
            return NavigationSuggestion(TwistCommand(), "IDLE", "NO_FRONT_LIDAR_POINTS")

        if front < self.front_clear_distance:
            return self._recovery(observation, front, left, right)

        linear = self.base_speed
        debug: Dict[str, float | str] = {"front": front}

        if left is not None and right is not None:
            corridor_width = left + right
            error = left - right
            if corridor_width < 0.9:
                linear = self.narrow_speed
            mode = "CORRIDOR_FOLLOW"
        elif left is not None:
            error = left - self.desired_wall_distance
            mode = "LEFT_WALL_FOLLOW"
        elif right is not None:
            error = self.desired_wall_distance - right
            mode = "RIGHT_WALL_FOLLOW"
        else:
            return self._recovery(observation, front, left, right)

        dt = max(0.02, observation.dt)
        d_error = 0.0 if self._last_error is None else (error - self._last_error) / dt
        self._last_error = error
        yaw_pd = self.kp * error + self.kd * d_error
        yaw_avoid = 0.0

        if front_left is not None and front_right is not None:
            debug["front_left"] = front_left
            debug["front_right"] = front_right
            if front_left < self.front_corner_avoid_distance:
                pressure = (self.front_corner_avoid_distance - front_left) / self.front_corner_avoid_distance
                yaw_avoid -= self.avoidance_gain * pressure
                linear = min(linear, self.narrow_speed)
                debug["front_left_pressure"] = pressure
            if front_right < self.front_corner_avoid_distance:
                pressure = (self.front_corner_avoid_distance - front_right) / self.front_corner_avoid_distance
                yaw_avoid += self.avoidance_gain * pressure
                linear = min(linear, self.narrow_speed)
                debug["front_right_pressure"] = pressure

        if left is not None and left < self.side_avoid_distance:
            pressure = (self.side_avoid_distance - left) / self.side_avoid_distance
            yaw_avoid -= self.avoidance_gain * pressure
            linear = min(linear, self.narrow_speed)
            debug["left_side_pressure"] = pressure
        if right is not None and right < self.side_avoid_distance:
            pressure = (self.side_avoid_distance - right) / self.side_avoid_distance
            yaw_avoid += self.avoidance_gain * pressure
            linear = min(linear, self.narrow_speed)
            debug["right_side_pressure"] = pressure

        yaw = yaw_pd + yaw_avoid
        yaw = max(-self.max_yaw, min(self.max_yaw, yaw))

        if front_left is not None and front_left < self.side_avoid_distance and yaw > 0.0:
            yaw = min(yaw, 0.0)
            debug["yaw_veto"] = "left_front_close"
        if front_right is not None and front_right < self.side_avoid_distance and yaw < 0.0:
            yaw = max(yaw, 0.0)
            debug["yaw_veto"] = "right_front_close"

        debug.update(
            {
                "error": error,
                "d_error": d_error,
                "yaw_pd": yaw_pd,
                "yaw_avoid": yaw_avoid,
                "control_sign": "positive_error_turns_left_negative_error_turns_right",
            }
        )
        return NavigationSuggestion(
            TwistCommand(linear, yaw),
            mode,
            "FRONT_CLEAR",
            debug,
        )

    def _recovery(
        self,
        observation: NavigationObservation,
        front: float | None,
        left: float | None,
        right: float | None,
    ) -> NavigationSuggestion:
        gap = largest_free_gap(
            observation.sectors.points,
            min_clearance_m=self.recovery_clearance,
            min_width_deg=20.0,
        )
        if gap is not None:
            yaw = max(-self.max_yaw, min(self.max_yaw, gap.center_deg / 70.0))
            forward = self.narrow_speed if abs(gap.center_deg) < 18.0 and front and front > 0.42 else 0.0
            return NavigationSuggestion(
                TwistCommand(forward, yaw),
                "RECOVERY",
                "FRONT_BLOCKED_SELECT_FREE_GAP",
                {
                    "gap_start": gap.start_deg,
                    "gap_end": gap.end_deg,
                    "gap_center": gap.center_deg,
                    "gap_width": gap.width_deg,
                    "front": front if front is not None else -1.0,
                },
            )

        turn_left_score = left if left is not None else 0.0
        turn_right_score = right if right is not None else 0.0
        yaw = self.max_yaw * 0.45 if turn_left_score >= turn_right_score else -self.max_yaw * 0.45
        return NavigationSuggestion(
            TwistCommand(0.0, yaw),
            "RECOVERY",
            "NO_CLEAR_GAP_TURN_TOWARD_OPEN_SIDE",
            {
                "left": turn_left_score,
                "right": turn_right_score,
                "front": front if front is not None else -1.0,
            },
        )


def create_navigation_module(name: str, **kwargs) -> NavigationModule:
    normalized = (name or "wall_follow").strip().lower()
    if normalized in ("wall_follow", "wall_following", "corridor"):
        return WallFollowNavigation(**kwargs)
    raise ValueError(f"Unknown navigation module: {name}")
