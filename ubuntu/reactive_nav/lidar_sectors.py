#!/usr/bin/env python3
"""LaserScan preprocessing and sector extraction for reactive navigation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SECTOR_DEGREES: Dict[str, Tuple[float, float]] = {
    "front_center": (-10.0, 10.0),
    "front": (-20.0, 20.0),
    "front_left": (20.0, 70.0),
    "front_right": (-70.0, -20.0),
    "left": (70.0, 110.0),
    "right": (-110.0, -70.0),
    "rear_left": (150.0, 180.0),
    "rear_right": (-180.0, -150.0),
}


@dataclass(frozen=True)
class SectorStats:
    name: str
    min_range: Optional[float]
    robust_min_range: Optional[float]
    median_range: Optional[float]
    valid_count: int

    @property
    def distance(self) -> Optional[float]:
        return self.robust_min_range if self.robust_min_range is not None else self.min_range


@dataclass(frozen=True)
class ScanPoint:
    angle_deg: float
    distance_m: float


@dataclass(frozen=True)
class SectorMap:
    sectors: Dict[str, SectorStats]
    points: Tuple[ScanPoint, ...]
    range_min: float
    range_max: float
    valid_count: int
    total_count: int

    def distance(self, name: str, default: Optional[float] = None) -> Optional[float]:
        stats = self.sectors.get(name)
        if stats is None:
            return default
        value = stats.distance
        return default if value is None else value

    @property
    def rear_distance(self) -> Optional[float]:
        values = [
            self.distance("rear_left"),
            self.distance("rear_right"),
        ]
        finite = [v for v in values if v is not None]
        return min(finite) if finite else None


def _percentile(values: Sequence[float], percentile: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * percentile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def normalize_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


def _angle_in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    angle_deg = normalize_angle_deg(angle_deg)
    start_deg = normalize_angle_deg(start_deg)
    end_deg = normalize_angle_deg(end_deg)
    if start_deg <= end_deg:
        return start_deg <= angle_deg <= end_deg
    return angle_deg >= start_deg or angle_deg <= end_deg


def _clean_range(raw: float, range_min: float, range_max: float) -> Optional[float]:
    if raw is None or math.isnan(raw):
        return None
    if math.isinf(raw):
        return float(range_max) if raw > 0 else None
    if raw < range_min:
        return None
    return min(float(raw), float(range_max))


def scan_to_points(scan, *, angle_offset_deg: float = 0.0) -> Tuple[ScanPoint, ...]:
    """Convert a ROS-like LaserScan object into cleaned angle/range points.

    ``angle_offset_deg`` rotates scan-frame angles into the robot base frame.
    Use it when the LiDAR frame is mounted yawed relative to the TurtleBot
    forward axis, for example +90 degrees when the LiDAR's zero angle points
    toward the robot's left side.
    """
    ranges = list(getattr(scan, "ranges", []) or [])
    range_min = float(getattr(scan, "range_min", 0.0) or 0.0)
    range_max = float(getattr(scan, "range_max", 12.0) or 12.0)
    angle_min = float(getattr(scan, "angle_min", 0.0) or 0.0)
    angle_increment = float(getattr(scan, "angle_increment", 0.0) or 0.0)

    points: List[ScanPoint] = []
    for index, raw in enumerate(ranges):
        cleaned = _clean_range(float(raw), range_min, range_max)
        if cleaned is None:
            continue
        angle_deg = math.degrees(angle_min + index * angle_increment) + angle_offset_deg
        points.append(ScanPoint(normalize_angle_deg(angle_deg), cleaned))
    return tuple(points)


def _stats_for_points(name: str, values: Iterable[float]) -> SectorStats:
    values = list(values)
    if not values:
        return SectorStats(name, None, None, None, 0)
    return SectorStats(
        name=name,
        min_range=min(values),
        robust_min_range=_percentile(values, 0.10),
        median_range=median(values),
        valid_count=len(values),
    )


def extract_sectors(
    scan,
    sector_degrees: Dict[str, Tuple[float, float]] = SECTOR_DEGREES,
    *,
    angle_offset_deg: float = 0.0,
) -> SectorMap:
    """Build robust min/median distances for each navigation sector."""
    points = scan_to_points(scan, angle_offset_deg=angle_offset_deg)
    sectors: Dict[str, SectorStats] = {}
    for name, (start_deg, end_deg) in sector_degrees.items():
        sector_values = [
            point.distance_m
            for point in points
            if _angle_in_sector(point.angle_deg, start_deg, end_deg)
        ]
        sectors[name] = _stats_for_points(name, sector_values)

    return SectorMap(
        sectors=sectors,
        points=points,
        range_min=float(getattr(scan, "range_min", 0.0) or 0.0),
        range_max=float(getattr(scan, "range_max", 12.0) or 12.0),
        valid_count=len(points),
        total_count=len(list(getattr(scan, "ranges", []) or [])),
    )


@dataclass(frozen=True)
class FreeGap:
    start_deg: float
    end_deg: float
    center_deg: float
    width_deg: float
    score: float
    min_distance_m: float


def largest_free_gap(
    points: Sequence[ScanPoint],
    min_clearance_m: float,
    min_width_deg: float = 18.0,
    search_min_deg: float = -120.0,
    search_max_deg: float = 120.0,
) -> Optional[FreeGap]:
    """Find a contiguous safe angular gap in cleaned scan points."""
    candidates = [
        point
        for point in points
        if search_min_deg <= point.angle_deg <= search_max_deg
    ]
    if not candidates:
        return None
    candidates = sorted(candidates, key=lambda point: point.angle_deg)

    gaps: List[List[ScanPoint]] = []
    current: List[ScanPoint] = []
    previous_angle: Optional[float] = None
    for point in candidates:
        safe = point.distance_m >= min_clearance_m
        contiguous = previous_angle is None or abs(point.angle_deg - previous_angle) <= 6.0
        if safe and contiguous:
            current.append(point)
        else:
            if current:
                gaps.append(current)
            current = [point] if safe else []
        previous_angle = point.angle_deg
    if current:
        gaps.append(current)

    best: Optional[FreeGap] = None
    for gap_points in gaps:
        start = gap_points[0].angle_deg
        end = gap_points[-1].angle_deg
        width = abs(end - start)
        if width < min_width_deg:
            continue
        center = (start + end) / 2.0
        min_distance = min(point.distance_m for point in gap_points)
        forward_bias = max(0.0, 1.0 - abs(center) / 120.0)
        score = width * 0.75 + min_distance * 15.0 + forward_bias * 25.0
        candidate = FreeGap(start, end, center, width, score, min_distance)
        if best is None or candidate.score > best.score:
            best = candidate
    return best
