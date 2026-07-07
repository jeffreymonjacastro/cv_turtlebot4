#!/usr/bin/env python3
"""Replaceable navigation modules for TurtleBot4 reactive navigation."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Dict, Protocol

try:
    from .lidar_sectors import SectorMap, largest_free_gap, traversable_gaps
except ImportError:  # pragma: no cover - direct script fallback
    from lidar_sectors import SectorMap, largest_free_gap, traversable_gaps


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
        base_speed: float = 0.14,
        narrow_speed: float = 0.08,
        max_yaw: float = 0.65,
        kp: float = 0.22,
        kd: float = 0.02,
        desired_wall_distance: float = 0.42,
        front_clear_distance: float = 0.55,
        recovery_clearance: float = 0.42,
        balance_deadband: float = 0.12,
    ):
        self.base_speed = base_speed
        self.narrow_speed = narrow_speed
        self.max_yaw = max_yaw
        self.kp = kp
        self.kd = kd
        self.desired_wall_distance = desired_wall_distance
        self.front_clear_distance = front_clear_distance
        self.recovery_clearance = recovery_clearance
        self.balance_deadband = max(0.0, balance_deadband)
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
        if mode == "CORRIDOR_FOLLOW" and abs(error) < self.balance_deadband:
            error = 0.0
        d_error = 0.0 if self._last_error is None else (error - self._last_error) / dt
        self._last_error = error
        yaw = self.kp * error + self.kd * d_error
        yaw = max(-self.max_yaw, min(self.max_yaw, yaw))

        if front_left is not None and front_right is not None:
            debug["front_left"] = front_left
            debug["front_right"] = front_right
            if front_left < 0.38 and yaw > 0.0:
                yaw = min(yaw, 0.0)
                debug["yaw_veto"] = "left_front_close"
            if front_right < 0.38 and yaw < 0.0:
                yaw = max(yaw, 0.0)
                debug["yaw_veto"] = "right_front_close"

        debug.update({"error": error, "d_error": d_error})
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


class ForwardAvoidNavigation:
    """Forward-first obstacle avoidance.

    This module does not center in corridors or follow a wall. It drives forward
    while the front is clear, then steers toward the side with more clearance.
    """

    name = "forward_avoid"

    def __init__(
        self,
        base_speed: float = 0.14,
        narrow_speed: float = 0.08,
        max_yaw: float = 0.65,
        front_clear_distance: float = 0.55,
        recovery_clearance: float = 0.42,
        soft_avoid_distance: float = 0.58,
        soft_avoid_gain: float = 0.75,
        curve_heading_gain: float = 0.55,
        reverse_turn_cooldown_s: float = 2.0,
        robot_width_m: float = 0.36,
        footprint_margin_m: float = 0.08,
        footprint_lookahead_m: float = 0.85,
        path_forward_heading_tolerance_deg: float = 12.0,
        drive_heading_limit_deg: float = 38.0,
        local_path_heading_deg: float = 50.0,
        local_path_step_deg: float = 10.0,
        **_unused,
    ):
        self.base_speed = base_speed
        self.narrow_speed = narrow_speed
        self.max_yaw = max_yaw
        self.front_clear_distance = front_clear_distance
        self.recovery_clearance = recovery_clearance
        self.soft_avoid_distance = soft_avoid_distance
        self.soft_avoid_gain = soft_avoid_gain
        self.curve_heading_gain = max(0.0, curve_heading_gain)
        self.reverse_turn_cooldown_s = max(0.0, reverse_turn_cooldown_s)
        self.robot_width_m = robot_width_m
        self.footprint_margin_m = footprint_margin_m
        self.footprint_lookahead_m = footprint_lookahead_m
        self.path_forward_heading_tolerance_deg = max(3.0, path_forward_heading_tolerance_deg)
        self.drive_heading_limit_deg = max(self.path_forward_heading_tolerance_deg, drive_heading_limit_deg)
        self.local_path_heading_deg = max(10.0, local_path_heading_deg)
        self.local_path_step_deg = max(2.0, local_path_step_deg)
        self._last_turn_sign = 0.0
        self._last_turn_time = 0.0

    def compute(self, observation: NavigationObservation) -> NavigationSuggestion:
        sectors = observation.sectors
        if sectors.valid_count == 0:
            return NavigationSuggestion(TwistCommand(), "IDLE", "NO_VALID_LIDAR_POINTS")

        front = sectors.distance("front")
        front_center = sectors.distance("front_center")
        front_left = sectors.distance("front_left")
        front_right = sectors.distance("front_right")
        left = sectors.distance("left")
        right = sectors.distance("right")

        if front is None and front_center is None:
            return NavigationSuggestion(TwistCommand(), "IDLE", "NO_FRONT_LIDAR_POINTS")

        front_value = front if front is not None else front_center
        front_center_value = front_center if front_center is not None else front_value
        nearest_front = min(front_value, front_center_value)
        left_score = self._clearance_score(left, front_left)
        right_score = self._clearance_score(right, front_right)
        footprint = self._footprint_map(sectors)
        path = self._local_path_map(sectors)
        gap = self._best_traversable_gap(sectors)
        footprint_lane = footprint["lane_nearest_x"]
        side_delta = left_score - right_score
        preferred_turn = 1.0 if side_delta >= 0.0 else -1.0
        gap_center = gap.center_deg if gap is not None else path["best_heading_deg"]
        gap_clearance = gap.min_distance_m if gap is not None else path["best_clearance_m"]
        drive_heading = self._drive_heading(path, gap)
        debug = {
            "front": nearest_front,
            "left_score": left_score,
            "right_score": right_score,
            "side_delta": side_delta,
            "footprint_lane_x": footprint_lane if footprint_lane is not None else -1.0,
            "footprint_side_bias": footprint["side_bias"],
            "path_best_heading_deg": path["best_heading_deg"],
            "path_best_clearance_m": path["best_clearance_m"],
            "path_straight_clearance_m": path["straight_clearance_m"],
            "gap_center_deg": gap_center,
            "gap_clearance_m": gap_clearance,
            "drive_heading_deg": drive_heading,
            "drive_heading_limit_deg": self.drive_heading_limit_deg,
        }

        straight_path_ready = (
            path["straight_clearance_m"] >= self.footprint_lookahead_m
            and nearest_front >= self.front_clear_distance
            and (footprint_lane is None or footprint_lane >= self.front_clear_distance)
        )
        if straight_path_ready:
            soft_yaw = self._soft_obstacle_yaw(front_left, front_right, left, right)
            curve_yaw = self._curve_heading_yaw(drive_heading, soft_yaw)
            yaw = self._avoid_reverse_turn(soft_yaw + curve_yaw, observation.now)
            reason = "FRONT_CLEAR_ADAPTIVE_CURVE_STEER" if abs(yaw) > 0.02 else "FRONT_CLEAR_GO_STRAIGHT"
            return NavigationSuggestion(TwistCommand(self.base_speed, yaw), "FORWARD_AVOID", reason, debug)

        drive_path_ready = (
            abs(drive_heading) <= self.drive_heading_limit_deg
            and path["best_clearance_m"] >= self.front_clear_distance
            and nearest_front >= self.front_clear_distance * 0.8
        )
        if drive_path_ready:
            soft_yaw = self._soft_obstacle_yaw(front_left, front_right, left, right)
            yaw = self._avoid_reverse_turn(self._heading_to_yaw(drive_heading) + soft_yaw * 0.5, observation.now)
            return NavigationSuggestion(
                TwistCommand(self.narrow_speed, yaw),
                "FORWARD_AVOID",
                "DRIVE_DIAGONAL_TO_OPEN_SPACE",
                debug,
            )

        if gap is not None and abs(gap.center_deg) > self.drive_heading_limit_deg:
            yaw = self._heading_to_yaw(gap.center_deg)
            return NavigationSuggestion(
                TwistCommand(0.0, yaw),
                "AVOID_OBSTACLE",
                "TURN_IN_PLACE_TO_TRAVERSABLE_GAP",
                debug,
            )

        if path["straight_clearance_m"] < self.footprint_lookahead_m:
            yaw = self._heading_to_yaw(drive_heading)
            forward = self.narrow_speed if (
                abs(drive_heading) <= self.drive_heading_limit_deg
                and path["straight_clearance_m"] > self.front_clear_distance
            ) else 0.0
            return NavigationSuggestion(
                TwistCommand(forward, yaw),
                "AVOID_OBSTACLE",
                "LOCAL_PATH_SELECT_HEADING",
                debug,
            )

        free_gap = largest_free_gap(
            sectors.points,
            min_clearance_m=self.recovery_clearance,
            min_width_deg=20.0,
        )
        if free_gap is not None:
            yaw = max(-self.max_yaw, min(self.max_yaw, free_gap.center_deg / 90.0))
            debug.update(
                {
                    "gap_start": free_gap.start_deg,
                    "gap_end": free_gap.end_deg,
                    "gap_center": free_gap.center_deg,
                    "gap_width": free_gap.width_deg,
                }
            )
            return NavigationSuggestion(
                TwistCommand(0.0, yaw),
                "AVOID_OBSTACLE",
                "FRONT_NOT_CLEAR_TURN_TO_FREE_GAP",
                debug,
            )

        yaw = preferred_turn * self.max_yaw
        return NavigationSuggestion(
            TwistCommand(0.0, yaw),
            "AVOID_OBSTACLE",
            "NO_GAP_TURN_TO_MORE_OPEN_SIDE",
            debug,
        )

    def _clearance_score(self, side: float | None, front_side: float | None) -> float:
        values = [value for value in (side, front_side) if value is not None]
        return min(values) if values else 0.0

    def _soft_obstacle_yaw(
        self,
        front_left: float | None,
        front_right: float | None,
        left: float | None,
        right: float | None,
    ) -> float:
        left_risk = self._risk(front_left) + self._risk(left) * 0.45
        right_risk = self._risk(front_right) + self._risk(right) * 0.45
        yaw = self.soft_avoid_gain * (right_risk - left_risk)
        return max(-self.max_yaw, min(self.max_yaw, yaw))

    def _risk(self, distance: float | None) -> float:
        if distance is None or self.soft_avoid_distance <= 0.0:
            return 0.0
        return max(0.0, (self.soft_avoid_distance - distance) / self.soft_avoid_distance)

    def _footprint_map(self, sectors: SectorMap) -> Dict[str, float | None]:
        half_width = max(0.05, self.robot_width_m * 0.5)
        swept_half_width = half_width + max(0.0, self.footprint_margin_m)
        lookahead = max(0.20, self.footprint_lookahead_m)
        lane_nearest_x = None
        left_risk = 0.0
        right_risk = 0.0

        for point in sectors.points:
            angle = math.radians(point.angle_deg)
            x = point.distance_m * math.cos(angle)
            y = point.distance_m * math.sin(angle)
            if x <= 0.0 or x > lookahead:
                continue
            risk = max(0.0, (lookahead - x) / lookahead)
            if y >= 0.0:
                left_risk += risk
            else:
                right_risk += risk
            if abs(y) <= swept_half_width:
                if lane_nearest_x is None or x < lane_nearest_x:
                    lane_nearest_x = x

        side_bias = 0.0
        total_risk = left_risk + right_risk
        if total_risk > 0.0:
            side_bias = (right_risk - left_risk) / total_risk
        return {
            "lane_nearest_x": lane_nearest_x,
            "side_bias": side_bias,
            "left_risk": left_risk,
            "right_risk": right_risk,
        }

    def _footprint_steer_yaw(self, footprint: Dict[str, float | None], *, fallback: float) -> float:
        side_bias = float(footprint.get("side_bias") or 0.0)
        if abs(side_bias) < 0.05:
            side_bias = fallback
        return max(-self.max_yaw, min(self.max_yaw, side_bias * self.max_yaw))

    def _local_path_map(self, sectors: SectorMap) -> Dict[str, float]:
        headings = self._candidate_headings()
        best_heading = 0.0
        best_clearance = -1.0
        best_score = -1.0
        straight_clearance = self._path_clearance(sectors, 0.0)
        for heading in headings:
            clearance = self._path_clearance(sectors, heading)
            score = clearance - abs(heading) * 0.003
            if score > best_score:
                best_score = score
                best_heading = heading
                best_clearance = clearance
        return {
            "best_heading_deg": best_heading,
            "best_clearance_m": best_clearance,
            "straight_clearance_m": straight_clearance,
        }

    def _best_traversable_gap(self, sectors: SectorMap):
        gaps = traversable_gaps(
            sectors.points,
            robot_width_m=self.robot_width_m,
            margin_m=self.footprint_margin_m,
            min_clearance_m=max(self.front_clear_distance, self.footprint_lookahead_m),
        )
        return gaps[0] if gaps else None

    def _candidate_headings(self) -> list[float]:
        max_heading = self.local_path_heading_deg
        step = self.local_path_step_deg
        headings = [0.0]
        count = int(max_heading // step)
        for index in range(1, count + 1):
            value = index * step
            headings.extend([value, -value])
        return headings

    def _path_clearance(self, sectors: SectorMap, heading_deg: float) -> float:
        half_width = max(0.05, self.robot_width_m * 0.5)
        swept_half_width = half_width + max(0.0, self.footprint_margin_m)
        lookahead = max(0.20, self.footprint_lookahead_m)
        heading = math.radians(heading_deg)
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        clearance = lookahead
        for point in sectors.points:
            angle = math.radians(point.angle_deg)
            x = point.distance_m * math.cos(angle)
            y = point.distance_m * math.sin(angle)
            along = x * cos_h + y * sin_h
            lateral = -x * sin_h + y * cos_h
            if along <= 0.0 or along > lookahead:
                continue
            if abs(lateral) <= swept_half_width:
                clearance = min(clearance, along)
        return clearance

    def _heading_to_yaw(self, heading_deg: float) -> float:
        if abs(heading_deg) < 1.0:
            return 0.0
        ratio = heading_deg / max(1.0, self.local_path_heading_deg)
        return max(-self.max_yaw, min(self.max_yaw, ratio * self.max_yaw))

    def _drive_heading(self, path: Dict[str, float], gap) -> float:
        path_heading = float(path.get("best_heading_deg") or 0.0)
        if gap is None:
            return path_heading
        gap_heading = float(gap.center_deg)
        if abs(gap_heading) <= self.drive_heading_limit_deg:
            return gap_heading
        return path_heading

    def _curve_heading_yaw(self, heading_deg: float, soft_yaw: float) -> float:
        if abs(heading_deg) <= self.path_forward_heading_tolerance_deg:
            return 0.0
        heading_yaw = self._heading_to_yaw(heading_deg)
        if abs(soft_yaw) > 0.02 and heading_yaw * soft_yaw < 0.0:
            return 0.0
        return heading_yaw * self.curve_heading_gain

    def _avoid_reverse_turn(self, yaw: float, now: float) -> float:
        if abs(yaw) < 0.02:
            return 0.0
        sign = 1.0 if yaw > 0.0 else -1.0
        if (
            self._last_turn_sign != 0.0
            and sign != self._last_turn_sign
            and now - self._last_turn_time < self.reverse_turn_cooldown_s
        ):
            yaw *= 0.35
        else:
            self._last_turn_sign = sign
            self._last_turn_time = now
        return max(-self.max_yaw, min(self.max_yaw, yaw))


def create_navigation_module(name: str, **kwargs) -> NavigationModule:
    normalized = (name or "forward_avoid").strip().lower()
    if normalized in ("forward_avoid", "avoid", "straight_avoid", "forward"):
        return ForwardAvoidNavigation(**kwargs)
    if normalized in ("wall_follow", "wall_following", "corridor"):
        return WallFollowNavigation(**kwargs)
    raise ValueError(f"Unknown navigation module: {name}")
