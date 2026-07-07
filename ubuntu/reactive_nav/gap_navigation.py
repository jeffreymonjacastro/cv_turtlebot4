#!/usr/bin/env python3
"""Follow-the-Gap and FOCM navigation modules.

These modules only compute local LiDAR-based command suggestions. The main
reactive navigator still owns safety arbitration and command publishing.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from .lidar_sectors import ScanPoint, SectorMap, normalize_angle_deg
    from .wall_following import NavigationObservation, NavigationSuggestion, TwistCommand
except ImportError:  # pragma: no cover - direct script fallback
    from lidar_sectors import ScanPoint, SectorMap, normalize_angle_deg
    from wall_following import NavigationObservation, NavigationSuggestion, TwistCommand


@dataclass(frozen=True)
class GapCandidate:
    start_index: int
    end_index: int
    start_deg: float
    end_deg: float
    center_deg: float
    best_heading_deg: float
    width_deg: float
    min_distance_m: float
    max_distance_m: float
    physical_width_m: float
    low_boundary: ScanPoint
    high_boundary: ScanPoint
    score: float


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _angle_delta_deg(a: float, b: float) -> float:
    return normalize_angle_deg(a - b)


def _angle_abs_delta_deg(a: float, b: float) -> float:
    return abs(_angle_delta_deg(a, b))


def _point_xy(point: ScanPoint) -> Tuple[float, float]:
    angle = math.radians(point.angle_deg)
    return point.distance_m * math.cos(angle), point.distance_m * math.sin(angle)


def _distance_between(a: ScanPoint, b: ScanPoint) -> float:
    ax, ay = _point_xy(a)
    bx, by = _point_xy(b)
    return math.hypot(ax - bx, ay - by)


def _points_in_search(
    points: Sequence[ScanPoint],
    *,
    search_min_deg: float,
    search_max_deg: float,
) -> List[ScanPoint]:
    candidates = [
        point
        for point in points
        if search_min_deg <= point.angle_deg <= search_max_deg
    ]
    return sorted(candidates, key=lambda point: point.angle_deg)


def _safe_mask_with_bubble(
    points: Sequence[ScanPoint],
    *,
    min_clearance_m: float,
    bubble_radius_m: float,
    max_ray_gap_deg: float,
    range_max_m: float,
) -> List[bool]:
    safe = [point.distance_m >= min_clearance_m for point in points]
    finite_obstacles = [point for point in points if point.distance_m < range_max_m * 0.98]
    if not finite_obstacles or bubble_radius_m <= 0.0:
        return safe

    closest = min(finite_obstacles, key=lambda point: point.distance_m)
    if closest.distance_m <= 0.0:
        bubble_half_angle = 180.0
    else:
        ratio = _clamp(bubble_radius_m / max(closest.distance_m, 1e-3), 0.0, 1.0)
        bubble_half_angle = math.degrees(math.asin(ratio))
    bubble_half_angle = max(bubble_half_angle, max_ray_gap_deg)

    for index, point in enumerate(points):
        if _angle_abs_delta_deg(point.angle_deg, closest.angle_deg) <= bubble_half_angle:
            safe[index] = False
    return safe


def _best_point_heading(
    gap_points: Sequence[ScanPoint],
    *,
    center_deg: float,
    distance_score_cap_m: float,
) -> float:
    if not gap_points:
        return center_deg

    best = gap_points[0]
    best_score = -math.inf
    for point in gap_points:
        distance_score = min(point.distance_m, distance_score_cap_m)
        center_bias = max(0.0, 1.0 - _angle_abs_delta_deg(point.angle_deg, center_deg) / 80.0)
        forward_bias = max(0.0, 1.0 - abs(point.angle_deg) / 120.0)
        score = distance_score * 10.0 + center_bias * 4.0 + forward_bias * 2.0
        if score > best_score:
            best = point
            best_score = score
    return best.angle_deg


def _extract_gaps(
    sectors: SectorMap,
    *,
    min_clearance_m: float,
    bubble_radius_m: float,
    min_width_deg: float,
    min_physical_width_m: float,
    search_min_deg: float,
    search_max_deg: float,
    max_ray_gap_deg: float,
    distance_score_cap_m: float,
) -> List[GapCandidate]:
    points = _points_in_search(
        sectors.points,
        search_min_deg=search_min_deg,
        search_max_deg=search_max_deg,
    )
    if not points:
        return []

    safe = _safe_mask_with_bubble(
        points,
        min_clearance_m=min_clearance_m,
        bubble_radius_m=bubble_radius_m,
        max_ray_gap_deg=max_ray_gap_deg,
        range_max_m=sectors.range_max,
    )

    gaps: List[GapCandidate] = []
    start_index: Optional[int] = None
    for index, is_safe in enumerate(safe + [False]):
        if is_safe and start_index is None:
            start_index = index
            continue
        if is_safe:
            previous = points[index - 1]
            current = points[index]
            if abs(current.angle_deg - previous.angle_deg) <= max_ray_gap_deg:
                continue
            end_index = index - 1
        elif start_index is None:
            continue
        else:
            end_index = index - 1

        if start_index is None:
            continue
        gap_points = points[start_index : end_index + 1]
        if not gap_points:
            start_index = None
            continue

        start_deg = gap_points[0].angle_deg
        end_deg = gap_points[-1].angle_deg
        width_deg = end_deg - start_deg
        if width_deg >= min_width_deg:
            low_boundary = points[max(0, start_index - 1)]
            high_boundary = points[min(len(points) - 1, end_index + 1)]
            physical_width = _distance_between(low_boundary, high_boundary)
            if physical_width >= min_physical_width_m:
                center_deg = (start_deg + end_deg) / 2.0
                min_distance = min(point.distance_m for point in gap_points)
                max_distance = max(point.distance_m for point in gap_points)
                best_heading = _best_point_heading(
                    gap_points,
                    center_deg=center_deg,
                    distance_score_cap_m=distance_score_cap_m,
                )
                forward_bias = max(0.0, 1.0 - abs(best_heading) / 120.0)
                score = width_deg * 0.75 + physical_width * 35.0 + forward_bias * 20.0
                gaps.append(
                    GapCandidate(
                        start_index=start_index,
                        end_index=end_index,
                        start_deg=start_deg,
                        end_deg=end_deg,
                        center_deg=center_deg,
                        best_heading_deg=best_heading,
                        width_deg=width_deg,
                        min_distance_m=min_distance,
                        max_distance_m=max_distance,
                        physical_width_m=physical_width,
                        low_boundary=low_boundary,
                        high_boundary=high_boundary,
                        score=score,
                    )
                )
        start_index = index if is_safe else None
    return gaps


def _fallback_turn(
    sectors: SectorMap,
    *,
    max_yaw: float,
    front: float | None,
    reason: str,
) -> NavigationSuggestion:
    left = sectors.distance("left", 0.0) or 0.0
    right = sectors.distance("right", 0.0) or 0.0
    yaw = max_yaw * 0.45 if left >= right else -max_yaw * 0.45
    return NavigationSuggestion(
        TwistCommand(0.0, yaw),
        "RECOVERY",
        reason,
        {
            "left": left,
            "right": right,
            "front": front if front is not None else -1.0,
        },
    )


class FollowGapNavigation:
    """F1TENTH-style Follow-the-Gap local navigator.

    The implementation follows the practical lab variant: preprocess scan,
    place a safety bubble around the nearest obstacle, find the largest safe
    angular gap, then steer toward the best point inside that gap.
    """

    name = "follow_gap"

    def __init__(
        self,
        *,
        base_speed: float = 0.10,
        narrow_speed: float = 0.06,
        turn_slow_speed: float = 0.07,
        turn_slow_yaw_threshold: float = 22.0,
        max_yaw: float = 0.65,
        front_clear_distance: float = 0.55,
        slow_distance: float = 0.55,
        recovery_clearance: float = 0.42,
        gap_bubble_radius_m: float = 0.30,
        gap_min_width_deg: float = 18.0,
        gap_search_min_deg: float = -120.0,
        gap_search_max_deg: float = 120.0,
        gap_heading_scale_deg: float = 75.0,
        gap_distance_score_cap_m: float = 3.0,
        gap_forward_cone_deg: float = 18.0,
        robot_width_m: float = 0.36,
        gap_side_margin_m: float = 0.08,
        **_unused,
    ):
        self.base_speed = base_speed
        self.narrow_speed = narrow_speed
        self.turn_slow_speed = min(base_speed, max(0.0, turn_slow_speed))
        threshold = max(0.0, turn_slow_yaw_threshold)
        self.turn_slow_yaw_threshold = math.degrees(threshold) if threshold <= math.pi else threshold
        self.max_yaw = max_yaw
        self.front_clear_distance = front_clear_distance
        self.slow_distance = slow_distance
        self.recovery_clearance = recovery_clearance
        self.gap_bubble_radius_m = gap_bubble_radius_m
        self.gap_min_width_deg = gap_min_width_deg
        self.gap_search_min_deg = gap_search_min_deg
        self.gap_search_max_deg = gap_search_max_deg
        self.gap_heading_scale_deg = gap_heading_scale_deg
        self.gap_distance_score_cap_m = gap_distance_score_cap_m
        self.gap_forward_cone_deg = gap_forward_cone_deg
        self.min_physical_width_m = max(0.1, robot_width_m + 2.0 * gap_side_margin_m)

    def compute(self, observation: NavigationObservation) -> NavigationSuggestion:
        sectors = observation.sectors
        if sectors.valid_count == 0:
            return NavigationSuggestion(TwistCommand(), "IDLE", "NO_VALID_LIDAR_POINTS")
        front = sectors.distance("front")
        gaps = _extract_gaps(
            sectors,
            min_clearance_m=self.recovery_clearance,
            bubble_radius_m=self.gap_bubble_radius_m,
            min_width_deg=self.gap_min_width_deg,
            min_physical_width_m=self.min_physical_width_m,
            search_min_deg=self.gap_search_min_deg,
            search_max_deg=self.gap_search_max_deg,
            max_ray_gap_deg=6.0,
            distance_score_cap_m=self.gap_distance_score_cap_m,
        )
        if not gaps:
            return _fallback_turn(
                sectors,
                max_yaw=self.max_yaw,
                front=front,
                reason="FTG_NO_CLEAR_GAP_TURN_TOWARD_OPEN_SIDE",
            )

        selected = max(gaps, key=lambda gap: (gap.width_deg, gap.score))
        heading_deg = selected.best_heading_deg
        yaw = self.max_yaw * _clamp(heading_deg / self.gap_heading_scale_deg, -1.0, 1.0)
        linear = self._linear_speed(front, heading_deg, selected.min_distance_m)
        return NavigationSuggestion(
            TwistCommand(linear, yaw),
            "FOLLOW_GAP",
            "FTG_SELECT_MAX_ANGULAR_GAP",
            self._debug(selected, front, heading_deg, len(gaps)),
        )

    def _linear_speed(self, front: float | None, heading_deg: float, min_distance_m: float) -> float:
        if front is None:
            return 0.0
        if abs(heading_deg) > 60.0:
            return 0.0
        speed = self.base_speed
        if front < self.front_clear_distance and abs(heading_deg) > self.gap_forward_cone_deg:
            return 0.0
        if front < self.front_clear_distance or min_distance_m < self.front_clear_distance:
            speed = min(speed, self.narrow_speed)
        if front < self.slow_distance or min_distance_m < self.slow_distance:
            speed = min(speed, self.turn_slow_speed)
        if abs(heading_deg) > self.turn_slow_yaw_threshold:
            speed = min(speed, self.turn_slow_speed)
        turn_scale = max(0.55, 1.0 - abs(heading_deg) / 140.0)
        return max(0.0, min(speed, self.base_speed * turn_scale))

    def _debug(
        self,
        gap: GapCandidate,
        front: float | None,
        heading_deg: float,
        gap_count: int,
    ) -> Dict[str, float | str]:
        return {
            "gap_start": gap.start_deg,
            "gap_end": gap.end_deg,
            "gap_center": gap.center_deg,
            "gap_best_heading": heading_deg,
            "gap_width": gap.width_deg,
            "gap_physical_width_m": gap.physical_width_m,
            "gap_min_distance_m": gap.min_distance_m,
            "gap_max_distance_m": gap.max_distance_m,
            "gap_count": float(gap_count),
            "front": front if front is not None else -1.0,
            "algorithm": "follow_gap",
            "turn_slow_speed": self.turn_slow_speed,
        }


class FocmNavigation(FollowGapNavigation):
    """Follow the Obstacle Circle Method local navigator.

    This keeps FTG's reactive gap pipeline but selects gaps by physical width
    and computes the heading from obstacle-circle tangency around gap borders.
    """

    name = "focm"

    def __init__(
        self,
        *,
        focm_alpha: float = 40.0,
        focm_goal_heading_deg: float = 0.0,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.focm_alpha = max(0.0, focm_alpha)
        self.focm_goal_heading_deg = focm_goal_heading_deg

    def compute(self, observation: NavigationObservation) -> NavigationSuggestion:
        sectors = observation.sectors
        if sectors.valid_count == 0:
            return NavigationSuggestion(TwistCommand(), "IDLE", "NO_VALID_LIDAR_POINTS")
        front = sectors.distance("front")
        gaps = _extract_gaps(
            sectors,
            min_clearance_m=self.recovery_clearance,
            bubble_radius_m=self.gap_bubble_radius_m,
            min_width_deg=self.gap_min_width_deg,
            min_physical_width_m=self.min_physical_width_m,
            search_min_deg=self.gap_search_min_deg,
            search_max_deg=self.gap_search_max_deg,
            max_ray_gap_deg=6.0,
            distance_score_cap_m=self.gap_distance_score_cap_m,
        )
        if not gaps:
            return _fallback_turn(
                sectors,
                max_yaw=self.max_yaw,
                front=front,
                reason="FOCM_NO_CLEAR_GAP_TURN_TOWARD_OPEN_SIDE",
            )

        selected = max(gaps, key=lambda gap: (gap.physical_width_m, gap.score))
        avoid_heading, circle_debug = self._avoidance_heading(selected)
        closest_distance = min(selected.low_boundary.distance_m, selected.high_boundary.distance_m)
        weight = self.focm_alpha / max(closest_distance, 0.05)
        final_heading = (
            weight * avoid_heading + self.focm_goal_heading_deg
        ) / (weight + 1.0)
        final_heading = self._clamp_heading_to_gap(final_heading, selected)
        yaw = self.max_yaw * _clamp(final_heading / self.gap_heading_scale_deg, -1.0, 1.0)
        linear = self._linear_speed(front, final_heading, selected.min_distance_m)
        debug = self._debug(selected, front, final_heading, len(gaps))
        debug.update(circle_debug)
        debug["algorithm"] = "focm"
        debug["focm_avoid_heading"] = avoid_heading
        debug["focm_final_heading"] = final_heading
        debug["focm_heading_weight"] = weight
        return NavigationSuggestion(
            TwistCommand(linear, yaw),
            "FOCM",
            "FOCM_SELECT_WIDEST_PHYSICAL_GAP",
            debug,
        )

    def _avoidance_heading(self, gap: GapCandidate) -> Tuple[float, Dict[str, float | str]]:
        low_x, low_y = _point_xy(gap.low_boundary)
        high_x, high_y = _point_xy(gap.high_boundary)
        mid_x = (low_x + high_x) / 2.0
        mid_y = (low_y + high_y) / 2.0
        center_heading = math.degrees(math.atan2(mid_y, mid_x))
        radius = max(0.05, gap.physical_width_m / 2.0)

        low_dist = math.hypot(low_x, low_y)
        high_dist = math.hypot(high_x, high_y)
        if low_dist <= high_dist:
            cx, cy = low_x, low_y
            closest_label = "low_boundary"
            circle_distance = low_dist
        else:
            cx, cy = high_x, high_y
            closest_label = "high_boundary"
            circle_distance = high_dist

        if circle_distance > radius + 1e-6:
            base = math.degrees(math.atan2(cy, cx))
            delta = math.degrees(math.acos(_clamp(radius / circle_distance, -1.0, 1.0)))
            candidates = [
                normalize_angle_deg(base + delta),
                normalize_angle_deg(base - delta),
            ]
            heading = min(candidates, key=lambda angle: _angle_abs_delta_deg(angle, center_heading))
            case = "outside_obstacle_circle_tangent"
        else:
            base = math.degrees(math.atan2(cy, cx))
            candidates = [
                normalize_angle_deg(base + 90.0),
                normalize_angle_deg(base - 90.0),
            ]
            heading = min(candidates, key=lambda angle: _angle_abs_delta_deg(angle, center_heading))
            case = "inside_obstacle_circle_arc"

        return heading, {
            "focm_case": case,
            "focm_circle_center": closest_label,
            "focm_circle_radius_m": radius,
            "focm_circle_distance_m": circle_distance,
            "focm_gap_center_heading": center_heading,
        }

    def _clamp_heading_to_gap(self, heading_deg: float, gap: GapCandidate) -> float:
        margin = min(8.0, max(0.0, gap.width_deg / 5.0))
        low = gap.start_deg + margin
        high = gap.end_deg - margin
        if low > high:
            low, high = gap.start_deg, gap.end_deg
        return _clamp(heading_deg, low, high)
