#!/usr/bin/env python3
"""Replay deterministic synthetic LaserScan scenarios through reactive_nav.

This script is intentionally offline-only. It creates dependency-free
LaserScan-like objects, runs the real sector extraction, navigation module, and
behavior arbiter, then writes JSONL diagnostics. It never publishes /cmd_vel.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import random
import sys
import time
from typing import Any, Callable, Dict, Iterable, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ubuntu.reactive_nav.behavior_arbiter import (  # noqa: E402
    ArbiterInput,
    BehaviorArbiter,
    SignalState,
    SignDebouncer,
)
from ubuntu.reactive_nav.lidar_sectors import (  # noqa: E402
    SECTOR_DEGREES,
    SectorMap,
    extract_sectors,
    normalize_angle_deg,
)
from ubuntu.reactive_nav.qr_logger import QRLogger  # noqa: E402
from ubuntu.reactive_nav.reactive_navigator import load_profile_parameters  # noqa: E402
from ubuntu.reactive_nav.turn_controller import TurnController  # noqa: E402
from ubuntu.reactive_nav.wall_following import (  # noqa: E402
    NavigationObservation,
    TwistCommand,
    create_navigation_module,
)


ANGLE_MIN_DEG = -180
ANGLE_MAX_DEG = 180
ANGLE_STEP_DEG = 1.0
SCAN_SIZE = int((ANGLE_MAX_DEG - ANGLE_MIN_DEG) / ANGLE_STEP_DEG) + 1

DEFAULT_PROFILE: Dict[str, Any] = {
    "profile_name": "offline_default",
    "nav_module": "wall_follow",
    "base_speed": 0.10,
    "narrow_speed": 0.06,
    "turn_slow_speed": 0.07,
    "turn_slow_yaw_threshold": 0.24,
    "max_yaw": 0.65,
    "wall_kp": 0.45,
    "wall_kd": 0.04,
    "front_clear_distance": 0.55,
    "slow_distance": 0.55,
    "corner_slow_speed": 0.035,
    "recovery_clearance": 0.42,
    "side_avoid_distance": 0.34,
    "front_corner_avoid_distance": 0.62,
    "avoidance_gain": 0.65,
    "enable_corner_yaw_veto": True,
    "enable_corner_slowdown": True,
    "enable_side_yaw_veto": True,
    "enable_anti_spin": False,
    "anti_spin_yaw_threshold": 0.42,
    "anti_spin_linear_threshold": 0.025,
    "anti_spin_trigger_cycles": 8,
    "anti_spin_recovery_speed": 0.035,
    "angular_smoothing_alpha": 1.0,
    "gap_bubble_radius_m": 0.30,
    "gap_min_width_deg": 18.0,
    "gap_search_min_deg": -120.0,
    "gap_search_max_deg": 120.0,
    "gap_heading_scale_deg": 75.0,
    "gap_distance_score_cap_m": 3.0,
    "gap_forward_cone_deg": 18.0,
    "robot_width_m": 0.36,
    "gap_side_margin_m": 0.08,
    "focm_alpha": 40.0,
    "focm_goal_heading_deg": 0.0,
    "front_stop_distance": 0.32,
    "front_stop_clear_distance": 0.40,
    "side_stop_distance": 0.14,
    "side_stop_clear_distance": 0.20,
    "emergency_clear_cycles": 3,
    "turn_clearance": 0.42,
    "turn_speed": 0.45,
    "turn_degrees": 90.0,
    "settle_seconds": 0.25,
    "align_max_seconds": 1.6,
    "align_yaw_limit": 0.28,
    "align_gain": 0.45,
    "align_error_threshold": 0.08,
    "align_stable_cycles": 3,
    "align_same_direction_only": True,
    "max_scan_age_s": 0.50,
    "sign_confirm_window": 8,
    "sign_confirm_count": 5,
    "sign_min_confidence": 0.70,
    "sign_min_area_ratio": 0.03,
    "sign_cooldown_s": 3.0,
    "qr_hold_s": 0.8,
    "qr_confirm_count": 2,
}

DEFAULT_CONFIG_BY_MODULE = {
    "wall_follow": REPO_ROOT / "ubuntu/reactive_nav/configs/wall_follow_safe.yaml",
    "follow_gap": REPO_ROOT / "ubuntu/reactive_nav/configs/follow_gap_safe.yaml",
    "focm": REPO_ROOT / "ubuntu/reactive_nav/configs/focm_safe.yaml",
}


@dataclass
class FakeLaserScan:
    ranges: List[float]
    stamp: float = 0.0
    frame_id: str = "synthetic_laser"
    angle_min: float = math.radians(ANGLE_MIN_DEG)
    angle_max: float = math.radians(ANGLE_MAX_DEG)
    angle_increment: float = math.radians(ANGLE_STEP_DEG)
    range_min: float = 0.12
    range_max: float = 4.0


SignalFn = Callable[[float, float], SignalState]
QrFn = Callable[[float], Optional[Dict[str, Any]]]
ScanFn = Callable[[float, random.Random], FakeLaserScan]


@dataclass(frozen=True)
class Scenario:
    name: str
    duration_s: float
    scan_fn: ScanFn
    expected: Dict[str, Any]
    signal_fn: SignalFn = field(default=lambda _t, _now: SignalState())
    qr_fn: QrFn = field(default=lambda _t: None)
    stale_from_s: Optional[float] = None

    def scan_at(self, t: float, rng: random.Random) -> FakeLaserScan:
        return self.scan_fn(t, rng)

    def signal_at(self, t: float, now: float) -> SignalState:
        return self.signal_fn(t, now)

    def qr_at(self, t: float) -> Optional[Dict[str, Any]]:
        return self.qr_fn(t)

    def lidar_fresh_at(self, t: float) -> bool:
        return self.stale_from_s is None or t < self.stale_from_s


def _angle_in_span(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    angle_deg = normalize_angle_deg(angle_deg)
    start_deg = normalize_angle_deg(start_deg)
    end_deg = normalize_angle_deg(end_deg)
    if start_deg <= end_deg:
        return start_deg <= angle_deg <= end_deg
    return angle_deg >= start_deg or angle_deg <= end_deg


def _set_arc(ranges: List[float], start_deg: float, end_deg: float, value: float) -> None:
    for index in range(SCAN_SIZE):
        angle = ANGLE_MIN_DEG + index * ANGLE_STEP_DEG
        if _angle_in_span(angle, start_deg, end_deg):
            ranges[index] = value


def corridor_scan(
    *,
    front: float = 2.0,
    front_center: Optional[float] = None,
    front_left: float = 1.4,
    front_right: float = 1.4,
    left: float = 0.65,
    right: float = 0.65,
    rear: float = 1.0,
    default: float = 2.5,
    stamp: float = 0.0,
    range_max: float = 4.0,
) -> FakeLaserScan:
    """Create a simple corridor-like scan with sector-specific distances."""

    ranges = [default] * SCAN_SIZE
    _set_arc(ranges, 150.0, 180.0, rear)
    _set_arc(ranges, -180.0, -150.0, rear)
    _set_arc(ranges, 70.0, 110.0, left)
    _set_arc(ranges, -110.0, -70.0, right)
    _set_arc(ranges, 20.0, 70.0, front_left)
    _set_arc(ranges, -70.0, -20.0, front_right)
    _set_arc(ranges, -20.0, 20.0, front)
    if front_center is not None:
        _set_arc(ranges, -10.0, 10.0, front_center)
    return FakeLaserScan(ranges=ranges, stamp=stamp, range_max=range_max)


def all_invalid_scan(t: float, _rng: random.Random) -> FakeLaserScan:
    values = [math.nan] * SCAN_SIZE
    for index in range(0, SCAN_SIZE, 5):
        values[index] = 0.02
    for index in range(2, SCAN_SIZE, 11):
        values[index] = -1.0
    return FakeLaserScan(ranges=values, stamp=t)


def noisy_corridor_scan(t: float, rng: random.Random) -> FakeLaserScan:
    scan = corridor_scan(front=1.8, left=0.58, right=0.62, front_left=1.1, front_right=1.2, stamp=t)
    ranges = []
    for value in scan.ranges:
        if math.isfinite(value):
            ranges.append(max(scan.range_min, value + rng.uniform(-0.025, 0.025)))
        else:
            ranges.append(value)
    for index in range(0, SCAN_SIZE, 37):
        ranges[index] = math.nan
    for index in range(11, SCAN_SIZE, 43):
        ranges[index] = math.inf
    for index in range(19, SCAN_SIZE, 53):
        ranges[index] = scan.range_max + 10.0
    for index in range(23, SCAN_SIZE, 59):
        ranges[index] = 0.01
    return FakeLaserScan(ranges=ranges, stamp=t, range_max=scan.range_max)


def noisy_corridor_with_outliers_scan(t: float, rng: random.Random) -> FakeLaserScan:
    left = 0.52 + 0.04 * math.sin(t * 2.1)
    right = 0.58 + 0.05 * math.sin(t * 2.7 + 1.0)
    front_left = 0.82 + 0.05 * math.sin(t * 3.3)
    front_right = 0.86 + 0.05 * math.sin(t * 2.9 + 0.7)
    scan = corridor_scan(
        front=1.45 + 0.05 * math.sin(t * 1.7),
        left=left,
        right=right,
        front_left=front_left,
        front_right=front_right,
        stamp=t,
    )
    ranges = []
    for index, value in enumerate(scan.ranges):
        jitter = rng.uniform(-0.045, 0.045)
        ranges.append(max(scan.range_min, min(scan.range_max, value + jitter)))
        if (index + int(t * 10)) % 41 == 0:
            ranges[index] = math.nan
        elif (index + int(t * 10)) % 47 == 0:
            ranges[index] = math.inf
        elif (index + int(t * 10)) % 67 == 0:
            ranges[index] = 0.05
    return FakeLaserScan(ranges=ranges, stamp=t, range_max=scan.range_max)


def _constant_scan(**kwargs: Any) -> ScanFn:
    def build(t: float, _rng: random.Random) -> FakeLaserScan:
        return corridor_scan(stamp=t, **kwargs)

    return build


def _approach_corner_scan(*, side: str) -> ScanFn:
    def build(t: float, _rng: random.Random) -> FakeLaserScan:
        risk = max(0.24, 1.10 - 0.12 * t)
        front = max(0.62, 1.65 - 0.05 * t)
        if side == "left":
            return corridor_scan(
                front=front,
                front_left=risk,
                front_right=1.20,
                left=0.92,
                right=0.36,
                default=1.7,
                stamp=t,
            )
        return corridor_scan(
            front=front,
            front_left=1.20,
            front_right=risk,
            left=0.36,
            right=0.92,
            default=1.7,
            stamp=t,
        )

    return build


def _asymmetric_corridor_scan(*, close_side: str) -> ScanFn:
    def build(t: float, _rng: random.Random) -> FakeLaserScan:
        close = 0.23 + 0.03 * math.sin(t * 1.8)
        far = 0.92 + 0.05 * math.sin(t * 1.2 + 0.4)
        if close_side == "left":
            return corridor_scan(
                front=1.80,
                front_left=0.72,
                front_right=1.10,
                left=close,
                right=far,
                stamp=t,
            )
        return corridor_scan(
            front=1.80,
            front_left=1.10,
            front_right=0.72,
            left=far,
            right=close,
            stamp=t,
        )

    return build


def _wall_too_close_scan(*, side: str) -> ScanFn:
    def build(t: float, _rng: random.Random) -> FakeLaserScan:
        close = 0.135 + 0.015 * math.sin(t * 1.5)
        if side == "left":
            return corridor_scan(
                front=1.35,
                front_left=0.55,
                front_right=0.95,
                left=close,
                right=0.75,
                stamp=t,
            )
        return corridor_scan(
            front=1.35,
            front_left=0.95,
            front_right=0.55,
            left=0.75,
            right=close,
            stamp=t,
        )

    return build


def _u_shape_dead_end_scan(t: float, _rng: random.Random) -> FakeLaserScan:
    side_opening = min(0.58, 0.26 + max(0.0, t - 4.0) * 0.04)
    return corridor_scan(
        front=0.33,
        front_center=0.33,
        front_left=0.31,
        front_right=0.31,
        left=side_opening,
        right=0.30,
        rear=1.40,
        default=0.48,
        stamp=t,
    )


def _spin_trap_scan(t: float, _rng: random.Random) -> FakeLaserScan:
    # Alternating open-side bias in otherwise open space can expose controllers
    # that circle instead of stabilizing forward motion.
    bias = 0.16 if int(t / 0.5) % 2 == 0 else -0.16
    return corridor_scan(
        front=2.40,
        front_left=1.80,
        front_right=1.80,
        left=1.40 + bias,
        right=1.40 - bias,
        rear=2.0,
        default=2.6,
        stamp=t,
    )


def _oscillatory_corridor_scan(t: float, _rng: random.Random) -> FakeLaserScan:
    sign = 1.0 if int(t / 0.4) % 2 == 0 else -1.0
    return corridor_scan(
        front=1.65,
        front_left=0.88 - 0.08 * sign,
        front_right=0.88 + 0.08 * sign,
        left=0.52 + 0.18 * sign,
        right=0.52 - 0.18 * sign,
        rear=1.0,
        default=1.7,
        stamp=t,
    )


def _u_curve_corridor_scan(t: float, _rng: random.Random) -> FakeLaserScan:
    """Multi-phase U-shaped corridor.

    Phases (total ~20 s at base_speed ~0.09 m/s):
      0–4 s   straight corridor heading "forward"
      4–7 s   approach U-bend end wall; left opens
      7–13 s  inside the U-bend turning left ~180°
      13–16 s exit the bend; front clears on the return leg
      16–20 s straight corridor heading "back"

    Sector evolution is smooth so the dead-reckoned trajectory looks
    like a recognisable U when projected by the visualiser.
    """
    cw = 0.55          # corridor half-width (wall distance on each side)
    fw_far = 2.2       # front distance when corridor is clear ahead
    fw_close = 0.32    # front distance when facing end wall

    # ── Phase 1: straight corridor ──────────────────────────────────────
    if t < 4.0:
        front_dist = max(fw_far - 0.25 * t, 0.90)   # closing slowly
        return corridor_scan(
            front=front_dist, front_left=front_dist * 0.65,
            front_right=front_dist * 0.65,
            left=cw, right=cw, rear=1.0, stamp=t,
        )

    # ── Phase 2: approaching U-bend end wall, left opens ────────────────
    if t < 7.0:
        p = (t - 4.0) / 3.0                          # 0→1 through phase
        front_dist = max(0.90 - 0.58 * p, fw_close)
        left_open = cw + 1.2 * p                     # left side opens
        fl_open = 0.60 + 0.9 * p                     # front-left opens
        return corridor_scan(
            front=front_dist, front_left=fl_open,
            front_right=max(0.45, 0.60 - 0.15 * p),
            left=left_open, right=cw, rear=1.0, stamp=t,
        )

    # ── Phase 3: inside the U-bend (turning left) ──────────────────────
    if t < 13.0:
        p = (t - 7.0) / 6.0                          # 0→1 through bend
        # front sweeps from blocked → open → blocked → open as heading
        # rotates ~180° through the bend
        front_dist = 0.35 + 1.4 * abs(math.sin(math.pi * p))
        # left = outer wall of the bend, gets farther at mid-turn
        left_dist = 0.9 + 0.7 * math.sin(math.pi * p)
        # right = inner wall of the bend, stays close
        right_dist = 0.28 + 0.12 * math.sin(math.pi * p)
        fl = 0.50 + 1.0 * max(0, math.sin(math.pi * (p - 0.1)))
        fr = 0.30 + 0.4 * max(0, math.sin(math.pi * (p + 0.15)))
        return corridor_scan(
            front=front_dist, front_left=fl, front_right=fr,
            left=left_dist, right=right_dist, rear=0.8, stamp=t,
        )

    # ── Phase 4: exiting bend, front clears ─────────────────────────────
    if t < 16.0:
        p = (t - 13.0) / 3.0
        front_dist = 0.50 + 1.5 * p
        right_open = cw + 1.0 * (1.0 - p)            # right was open, closing
        return corridor_scan(
            front=front_dist,
            front_left=max(0.50, 0.80 - 0.30 * p),
            front_right=0.55 + 0.60 * p,
            left=cw, right=max(cw, right_open), rear=1.0, stamp=t,
        )

    # ── Phase 5: straight return corridor ───────────────────────────────
    return corridor_scan(
        front=fw_far, front_left=fw_far * 0.65,
        front_right=fw_far * 0.65,
        left=cw, right=cw, rear=1.0, stamp=t,
    )


def _dead_end_scan(t: float, _rng: random.Random) -> FakeLaserScan:
    open_left = min(1.55, 0.45 + 0.12 * t)
    return corridor_scan(
        front=0.34,
        front_center=0.34,
        front_left=open_left,
        front_right=0.34,
        left=open_left,
        right=0.30,
        rear=0.80,
        default=0.75,
        stamp=t,
    )


def _fresh_sign(
    direction: str,
    *,
    start_s: float,
    end_s: float,
    event_id: str,
) -> SignalFn:
    def signal(t: float, now: float) -> SignalState:
        if start_s <= t <= end_s:
            return SignalState(
                direction=direction.lower(),
                confidence=0.92,
                bbox_area_ratio=0.12,
                bbox_center_x_ratio=0.50,
                actionable=True,
                timestamp=now,
                stale=False,
                event_id=event_id,
                reason="synthetic_fresh",
            )
        return SignalState(timestamp=now, reason="synthetic_none")

    return signal


def _qr_visible(start_s: float = 1.0, end_s: float = 3.0) -> QrFn:
    def qr(t: float) -> Optional[Dict[str, Any]]:
        if start_s <= t <= end_s:
            return {
                "visible": True,
                "content": "checkpoint-alpha",
                "source": "synthetic_camera",
                "frame_id": f"synthetic_qr_{int(t * 10):04d}",
                "confidence": None,
            }
        return {"visible": False, "content": None}

    return qr


def build_scenarios(default_duration_s: float) -> Dict[str, Scenario]:
    return {
        "open_corridor": Scenario(
            name="open_corridor",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=2.1, left=0.66, right=0.66, front_left=1.5, front_right=1.5),
            expected={"positive_progress": True, "max_emergency_stops": 0},
        ),
        "narrow_corridor": Scenario(
            name="narrow_corridor",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.5, left=0.34, right=0.35, front_left=0.80, front_right=0.82),
            expected={"positive_progress": True, "max_emergency_stops": 0},
        ),
        "left_wall_close": Scenario(
            name="left_wall_close",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.8, left=0.18, right=0.88, front_left=0.95, front_right=1.2),
            expected={"yaw_sign": "negative", "max_emergency_stops": 0},
        ),
        "right_wall_close": Scenario(
            name="right_wall_close",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.8, left=0.88, right=0.18, front_left=1.2, front_right=0.95),
            expected={"yaw_sign": "positive", "max_emergency_stops": 0},
        ),
        "front_blocked": Scenario(
            name="front_blocked",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=0.22, front_center=0.22, left=0.80, right=0.80, front_left=0.85, front_right=0.85),
            expected={"requires_stop": True, "allow_emergency": True},
        ),
        "dead_end_recovery": Scenario(
            name="dead_end_recovery",
            duration_s=default_duration_s,
            scan_fn=_dead_end_scan,
            expected={"requires_recovery": True, "max_forward_when_blocked_mps": 0.02},
        ),
        "left_sign_open": Scenario(
            name="left_sign_open",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=2.0, left=0.75, right=0.75, front_left=1.2, front_right=1.2),
            signal_fn=_fresh_sign("left", start_s=0.5, end_s=2.0, event_id="left_sign_open"),
            expected={"turn_direction": "left", "turn_count": 1},
        ),
        "right_sign_open": Scenario(
            name="right_sign_open",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=2.0, left=0.75, right=0.75, front_left=1.2, front_right=1.2),
            signal_fn=_fresh_sign("right", start_s=0.5, end_s=2.0, event_id="right_sign_open"),
            expected={"turn_direction": "right", "turn_count": 1},
        ),
        "left_sign_blocked": Scenario(
            name="left_sign_blocked",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.2, left=0.70, right=0.70, front_left=0.30, front_right=1.2),
            signal_fn=_fresh_sign("left", start_s=0.5, end_s=2.5, event_id="left_sign_blocked"),
            expected={"turn_count": 0, "blocked_turn": True},
        ),
        "right_sign_blocked": Scenario(
            name="right_sign_blocked",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.2, left=0.70, right=0.70, front_left=1.2, front_right=0.30),
            signal_fn=_fresh_sign("right", start_s=0.5, end_s=2.5, event_id="right_sign_blocked"),
            expected={"turn_count": 0, "blocked_turn": True},
        ),
        "stale_lidar": Scenario(
            name="stale_lidar",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=2.0, left=0.70, right=0.70, front_left=1.2, front_right=1.2),
            stale_from_s=0.0,
            expected={"requires_stop": True, "stale_lidar": True},
        ),
        "all_invalid_lidar": Scenario(
            name="all_invalid_lidar",
            duration_s=default_duration_s,
            scan_fn=all_invalid_scan,
            expected={"requires_stop": True, "invalid_lidar": True},
        ),
        "noisy_lidar_nan_inf": Scenario(
            name="noisy_lidar_nan_inf",
            duration_s=default_duration_s,
            scan_fn=noisy_corridor_scan,
            expected={"positive_progress": True, "handles_invalid_ranges": True},
        ),
        "qr_visible": Scenario(
            name="qr_visible",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.7, left=0.65, right=0.65, front_left=1.1, front_right=1.1),
            qr_fn=_qr_visible(),
            expected={"qr_logged_count": 1},
        ),
        "repeated_sign_cooldown": Scenario(
            name="repeated_sign_cooldown",
            duration_s=max(default_duration_s, 8.0),
            scan_fn=_constant_scan(front=2.0, left=0.75, right=0.75, front_left=1.2, front_right=1.2),
            signal_fn=_fresh_sign("left", start_s=0.5, end_s=999.0, event_id="repeated_left_same_event"),
            expected={"turn_direction": "left", "max_turn_count": 1},
        ),
        "front_left_corner_blocked": Scenario(
            name="front_left_corner_blocked",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.25, front_left=0.42, front_right=1.35, left=1.05, right=0.30),
            expected={"corner_risk_count": 0, "front_left_risk": True, "avoid_positive_yaw": True},
        ),
        "front_right_corner_blocked": Scenario(
            name="front_right_corner_blocked",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.25, front_left=1.35, front_right=0.42, left=0.30, right=1.05),
            expected={"corner_risk_count": 0, "front_right_risk": True, "avoid_negative_yaw": True},
        ),
        "corner_left_approach": Scenario(
            name="corner_left_approach",
            duration_s=max(default_duration_s, 10.0),
            scan_fn=_approach_corner_scan(side="left"),
            expected={"corner_risk_count": 0, "slowdown_before_corner": True},
        ),
        "corner_right_approach": Scenario(
            name="corner_right_approach",
            duration_s=max(default_duration_s, 10.0),
            scan_fn=_approach_corner_scan(side="right"),
            expected={"corner_risk_count": 0, "slowdown_before_corner": True},
        ),
        "narrow_left_turn": Scenario(
            name="narrow_left_turn",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.10, front_left=0.34, front_right=0.80, left=0.34, right=0.50),
            signal_fn=_fresh_sign("left", start_s=0.5, end_s=2.5, event_id="narrow_left_turn"),
            expected={"turn_count": 0, "blocked_turn": True, "corner_risk_count": 0},
        ),
        "narrow_right_turn": Scenario(
            name="narrow_right_turn",
            duration_s=default_duration_s,
            scan_fn=_constant_scan(front=1.10, front_left=0.80, front_right=0.34, left=0.50, right=0.34),
            signal_fn=_fresh_sign("right", start_s=0.5, end_s=2.5, event_id="narrow_right_turn"),
            expected={"turn_count": 0, "blocked_turn": True, "corner_risk_count": 0},
        ),
        "asymmetric_corridor_left_close": Scenario(
            name="asymmetric_corridor_left_close",
            duration_s=default_duration_s,
            scan_fn=_asymmetric_corridor_scan(close_side="left"),
            expected={"side_risk_count": 0, "yaw_sign": "negative"},
        ),
        "asymmetric_corridor_right_close": Scenario(
            name="asymmetric_corridor_right_close",
            duration_s=default_duration_s,
            scan_fn=_asymmetric_corridor_scan(close_side="right"),
            expected={"side_risk_count": 0, "yaw_sign": "positive"},
        ),
        "wall_too_close_left": Scenario(
            name="wall_too_close_left",
            duration_s=default_duration_s,
            scan_fn=_wall_too_close_scan(side="left"),
            expected={"side_risk_count": 0, "turn_away": "right"},
        ),
        "wall_too_close_right": Scenario(
            name="wall_too_close_right",
            duration_s=default_duration_s,
            scan_fn=_wall_too_close_scan(side="right"),
            expected={"side_risk_count": 0, "turn_away": "left"},
        ),
        "u_shape_dead_end": Scenario(
            name="u_shape_dead_end",
            duration_s=max(default_duration_s, 10.0),
            scan_fn=_u_shape_dead_end_scan,
            expected={"requires_recovery": True, "recovery_loop_count_max": 1, "spin_ratio_max": 0.35},
        ),
        "spin_trap_open_space": Scenario(
            name="spin_trap_open_space",
            duration_s=max(default_duration_s, 10.0),
            scan_fn=_spin_trap_scan,
            expected={"positive_progress": True, "spin_ratio_max": 0.20, "oscillation_score_max": 35.0},
        ),
        "noisy_corridor_with_outliers": Scenario(
            name="noisy_corridor_with_outliers",
            duration_s=max(default_duration_s, 10.0),
            scan_fn=noisy_corridor_with_outliers_scan,
            expected={"positive_progress": True, "corner_risk_count": 0, "oscillation_score_max": 35.0},
        ),
        "oscillatory_corridor": Scenario(
            name="oscillatory_corridor",
            duration_s=max(default_duration_s, 10.0),
            scan_fn=_oscillatory_corridor_scan,
            expected={"oscillation_score_max": 35.0, "angular_sign_changes_per_min_max": 45.0},
        ),
        "u_curve_corridor": Scenario(
            name="u_curve_corridor",
            duration_s=20.0,
            scan_fn=_u_curve_corridor_scan,
            expected={"positive_progress": True, "corner_risk_count": 0, "spin_ratio_max": 0.25},
        ),
    }


def _coerce_float(params: Dict[str, Any], key: str) -> float:
    return float(params.get(key, DEFAULT_PROFILE[key]))


def _coerce_int(params: Dict[str, Any], key: str) -> int:
    return int(params.get(key, DEFAULT_PROFILE[key]))


def _coerce_bool(params: Dict[str, Any], key: str) -> bool:
    return bool(params.get(key, DEFAULT_PROFILE[key]))


def load_replay_profile(
    module_name: str,
    *,
    config_path: Optional[Path] = None,
    profile_name: Optional[str] = None,
) -> Dict[str, Any]:
    params = dict(DEFAULT_PROFILE)
    selected_config = config_path or DEFAULT_CONFIG_BY_MODULE.get(module_name)
    if selected_config and selected_config.exists():
        params.update(load_profile_parameters(selected_config))
        params["_config_path"] = str(selected_config)
    elif selected_config:
        params["_config_path"] = str(selected_config)
        params["_config_warning"] = "missing_config_using_defaults"

    params["nav_module"] = module_name
    if profile_name:
        params["profile_name"] = profile_name
    elif not params.get("profile_name"):
        params["profile_name"] = f"{module_name}_offline"
    return params


def nav_kwargs(params: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "base_speed": _coerce_float(params, "base_speed"),
        "narrow_speed": _coerce_float(params, "narrow_speed"),
        "turn_slow_speed": _coerce_float(params, "turn_slow_speed"),
        "turn_slow_yaw_threshold": _coerce_float(params, "turn_slow_yaw_threshold"),
        "max_yaw": _coerce_float(params, "max_yaw"),
        "kp": _coerce_float(params, "wall_kp"),
        "kd": _coerce_float(params, "wall_kd"),
        "front_clear_distance": _coerce_float(params, "front_clear_distance"),
        "slow_distance": _coerce_float(params, "slow_distance"),
        "recovery_clearance": _coerce_float(params, "recovery_clearance"),
        "side_avoid_distance": _coerce_float(params, "side_avoid_distance"),
        "front_corner_avoid_distance": _coerce_float(params, "front_corner_avoid_distance"),
        "avoidance_gain": _coerce_float(params, "avoidance_gain"),
        "gap_bubble_radius_m": _coerce_float(params, "gap_bubble_radius_m"),
        "gap_min_width_deg": _coerce_float(params, "gap_min_width_deg"),
        "gap_search_min_deg": _coerce_float(params, "gap_search_min_deg"),
        "gap_search_max_deg": _coerce_float(params, "gap_search_max_deg"),
        "gap_heading_scale_deg": _coerce_float(params, "gap_heading_scale_deg"),
        "gap_distance_score_cap_m": _coerce_float(params, "gap_distance_score_cap_m"),
        "gap_forward_cone_deg": _coerce_float(params, "gap_forward_cone_deg"),
        "robot_width_m": _coerce_float(params, "robot_width_m"),
        "gap_side_margin_m": _coerce_float(params, "gap_side_margin_m"),
        "focm_alpha": _coerce_float(params, "focm_alpha"),
        "focm_goal_heading_deg": _coerce_float(params, "focm_goal_heading_deg"),
    }


def build_arbiter(params: Dict[str, Any]) -> BehaviorArbiter:
    signs = SignDebouncer(
        confirm_window=_coerce_int(params, "sign_confirm_window"),
        confirm_count=_coerce_int(params, "sign_confirm_count"),
        min_confidence=_coerce_float(params, "sign_min_confidence"),
        min_area_ratio=_coerce_float(params, "sign_min_area_ratio"),
        cooldown_s=_coerce_float(params, "sign_cooldown_s"),
    )
    turns = TurnController(
        turn_speed=_coerce_float(params, "turn_speed"),
        turn_degrees=_coerce_float(params, "turn_degrees"),
        settle_seconds=_coerce_float(params, "settle_seconds"),
        align_max_seconds=_coerce_float(params, "align_max_seconds"),
        align_yaw_limit=_coerce_float(params, "align_yaw_limit"),
        align_gain=_coerce_float(params, "align_gain"),
        align_error_threshold=_coerce_float(params, "align_error_threshold"),
        align_stable_cycles=_coerce_int(params, "align_stable_cycles"),
        align_same_direction_only=_coerce_bool(params, "align_same_direction_only"),
    )
    return BehaviorArbiter(
        front_stop_distance=_coerce_float(params, "front_stop_distance"),
        front_stop_clear_distance=_coerce_float(params, "front_stop_clear_distance"),
        side_stop_distance=_coerce_float(params, "side_stop_distance"),
        side_stop_clear_distance=_coerce_float(params, "side_stop_clear_distance"),
        emergency_clear_cycles=_coerce_int(params, "emergency_clear_cycles"),
        slow_distance=_coerce_float(params, "slow_distance"),
        front_corner_avoid_distance=_coerce_float(params, "front_corner_avoid_distance"),
        corner_slow_speed=_coerce_float(params, "corner_slow_speed"),
        enable_corner_yaw_veto=_coerce_bool(params, "enable_corner_yaw_veto"),
        enable_corner_slowdown=_coerce_bool(params, "enable_corner_slowdown"),
        enable_side_yaw_veto=_coerce_bool(params, "enable_side_yaw_veto"),
        enable_anti_spin=_coerce_bool(params, "enable_anti_spin"),
        anti_spin_yaw_threshold=_coerce_float(params, "anti_spin_yaw_threshold"),
        anti_spin_linear_threshold=_coerce_float(params, "anti_spin_linear_threshold"),
        anti_spin_trigger_cycles=_coerce_int(params, "anti_spin_trigger_cycles"),
        anti_spin_recovery_speed=_coerce_float(params, "anti_spin_recovery_speed"),
        angular_smoothing_alpha=_coerce_float(params, "angular_smoothing_alpha"),
        qr_hold_s=_coerce_float(params, "qr_hold_s"),
        turn_clearance=_coerce_float(params, "turn_clearance"),
        sign_debouncer=signs,
        turn_controller=turns,
    )


def _json_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(numeric) or math.isinf(numeric):
        return None
    return numeric


def _json_clean(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_clean(item) for item in value]
    if isinstance(value, (str, bool)) or value is None:
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return _json_float(value)
    return str(value)


def _sector_record(sectors: Optional[SectorMap], lidar_age_s: float, lidar_fresh: bool) -> Dict[str, Any]:
    if sectors is None:
        distances = {name: None for name in SECTOR_DEGREES}
        counts = {name: 0 for name in SECTOR_DEGREES}
        raw_min = {name: None for name in SECTOR_DEGREES}
        valid_count = 0
        total_count = 0
    else:
        distances = {name: _json_float(stats.distance) for name, stats in sectors.sectors.items()}
        counts = {name: stats.valid_count for name, stats in sectors.sectors.items()}
        raw_min = {name: _json_float(stats.min_range) for name, stats in sectors.sectors.items()}
        valid_count = sectors.valid_count
        total_count = sectors.total_count

    left = distances.get("left")
    right = distances.get("right")
    return {
        "fresh": lidar_fresh,
        "age_s": _json_float(lidar_age_s),
        "valid_count": valid_count,
        "total_count": total_count,
        "front_center_m": distances.get("front_center"),
        "front_m": distances.get("front"),
        "front_left_m": distances.get("front_left"),
        "front_right_m": distances.get("front_right"),
        "left_m": left,
        "right_m": right,
        "rear_m": (
            _json_float(sectors.rear_distance)
            if sectors is not None and sectors.rear_distance is not None
            else None
        ),
        "left_minus_right_m": (
            _json_float(left - right)
            if isinstance(left, (int, float)) and isinstance(right, (int, float))
            else None
        ),
        "sector_distance_m": distances,
        "sector_raw_min_m": raw_min,
        "sector_valid_count": counts,
    }


def _safe_filename(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)


def _metadata_record(
    *,
    scenario: Scenario,
    module_name: str,
    params: Dict[str, Any],
    seed: int,
    dt_s: float,
    duration_s: float,
) -> Dict[str, Any]:
    return {
        "record_type": "metadata",
        "scenario": scenario.name,
        "profile_name": params.get("profile_name"),
        "nav_module": module_name,
        "seed": seed,
        "dt_s": dt_s,
        "duration_s": duration_s,
        "dry_run": True,
        "enable_motion": False,
        "command_publication": "disabled_offline_replay",
        "config": _json_clean({k: v for k, v in params.items() if not k.startswith("_")}),
        "config_path": params.get("_config_path"),
        "config_warning": params.get("_config_warning"),
        "expected": scenario.expected,
    }


def run_scenario(
    *,
    scenario: Scenario,
    module_name: str,
    params: Dict[str, Any],
    out_dir: Path,
    seed: int,
    dt_s: float,
    duration_s: Optional[float] = None,
) -> Path:
    duration = scenario.duration_s if duration_s is None else duration_s
    rng = random.Random(seed)
    nav_module = create_navigation_module(module_name, **nav_kwargs(params))
    arbiter = build_arbiter(params)
    profile_name = str(params.get("profile_name") or f"{module_name}_offline")
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = "__".join(
        [
            _safe_filename(scenario.name),
            _safe_filename(profile_name),
            _safe_filename(nav_module.name),
        ]
    )
    out_path = out_dir / f"{stem}.jsonl"
    qr_logger = QRLogger(
        out_dir / "qr_logs" / f"{stem}.qr.jsonl",
        confirm_count=_coerce_int(params, "qr_confirm_count"),
    )

    max_scan_age_s = _coerce_float(params, "max_scan_age_s")
    sim_start = time.monotonic()
    steps = max(1, int(math.ceil(duration / dt_s)))
    previous_state = ""

    with out_path.open("w", encoding="utf-8") as handle:
        handle.write(
            json.dumps(
                _metadata_record(
                    scenario=scenario,
                    module_name=nav_module.name,
                    params=params,
                    seed=seed,
                    dt_s=dt_s,
                    duration_s=duration,
                ),
                ensure_ascii=True,
                sort_keys=True,
            )
            + "\n"
        )

        for step_index in range(steps):
            t = min(duration, step_index * dt_s)
            now = sim_start + t
            scan = scenario.scan_at(t, rng)
            sectors = extract_sectors(scan)
            lidar_fresh = scenario.lidar_fresh_at(t)
            lidar_age_s = 0.0 if lidar_fresh else max_scan_age_s + dt_s + t

            nav_suggestion = None
            if lidar_fresh:
                nav_suggestion = nav_module.compute(NavigationObservation(sectors, now, dt_s))

            signal = scenario.signal_at(t, now)
            qr_payload = scenario.qr_at(t) or {"visible": False, "content": None}
            qr_event = None
            qr_recent = bool(qr_payload.get("visible"))
            if qr_payload.get("content"):
                qr_event = qr_logger.observe(
                    qr_payload.get("content"),
                    source=str(qr_payload.get("source") or "synthetic_camera"),
                    frame_id=qr_payload.get("frame_id"),
                    robot_state=previous_state,
                    confidence=qr_payload.get("confidence"),
                    context={
                        "scenario": scenario.name,
                        "timestamp_s": round(t, 3),
                        "nav_module": nav_module.name,
                    },
                )

            output = arbiter.decide(
                ArbiterInput(
                    sectors=sectors,
                    lidar_fresh=lidar_fresh,
                    nav_suggestion=nav_suggestion,
                    signal=signal,
                    qr_recent=qr_recent,
                    now=now,
                )
            )
            published_command = output.command
            requested_command = output.command

            output_debug = dict(output.debug) if output.debug else {}
            emergency_reason = (
                output.reason
                if output.state == "EMERGENCY_STOP"
                else str(output_debug.get("emergency_trigger_reason") or "NONE")
            )
            turn_debug = {
                key: value
                for key, value in output_debug.items()
                if key.startswith("turn_") or key.startswith("align_")
            }
            record = {
                "record_type": "step",
                "timestamp": round(t, 3),
                "time_s": round(t, 3),
                "dt_s": dt_s,
                "scenario": scenario.name,
                "profile_name": profile_name,
                "state": output.state,
                "previous_state": previous_state,
                "reason": output.reason,
                "nav": {
                    "module": nav_module.name,
                    "suggestion_mode": nav_suggestion.mode if nav_suggestion else None,
                    "suggestion_reason": nav_suggestion.reason if nav_suggestion else None,
                    "suggested_linear_x": (
                        _json_float(nav_suggestion.command.linear_x) if nav_suggestion else None
                    ),
                    "suggested_angular_z": (
                        _json_float(nav_suggestion.command.angular_z) if nav_suggestion else None
                    ),
                    "debug": _json_clean(nav_suggestion.debug if nav_suggestion else {}),
                },
                "arbiter_debug": _json_clean(output_debug),
                "turn": _json_clean(turn_debug),
                "emergency": {
                    "active": output.state == "EMERGENCY_STOP" or bool(output_debug.get("emergency_active")),
                    "reason": emergency_reason,
                    "trigger_count": _json_float(output_debug.get("emergency_trigger_count")),
                },
                "lidar": _sector_record(sectors, lidar_age_s, lidar_fresh),
                "freshness": {
                    "lidar_age_s": _json_float(lidar_age_s),
                    "lidar_fresh": lidar_fresh,
                    "signal_stale": signal.stale,
                },
                "signal": {
                    "direction": signal.direction,
                    "fresh": not signal.stale,
                    "stale": signal.stale,
                    "confidence": _json_float(signal.confidence),
                    "bbox_area_ratio": _json_float(signal.bbox_area_ratio),
                    "bbox_center_x_ratio": _json_float(signal.bbox_center_x_ratio),
                    "actionable": signal.actionable,
                    "reason": signal.reason,
                    "event_id": signal.event_id,
                },
                "qr": {
                    "visible": bool(qr_payload.get("visible")),
                    "content": qr_payload.get("content"),
                    "logged": bool(qr_event.logged) if qr_event else False,
                    "duplicate": bool(qr_event.duplicate) if qr_event else False,
                    "log_path": qr_event.path if qr_event else str(qr_logger.log_path),
                },
                "command": {
                    "requested_linear_x": _json_float(requested_command.linear_x),
                    "requested_angular_z": _json_float(requested_command.angular_z),
                    "published_linear_x": _json_float(published_command.linear_x),
                    "published_angular_z": _json_float(published_command.angular_z),
                    "positive_angular_z_means": "left_turn",
                    "motion_published_to_robot": False,
                    "publication_mode": "offline_would_publish_after_arbitration",
                },
                "mode_flags": {
                    "dry_run": True,
                    "enable_motion": False,
                    "motion_enabled_this_cycle": False,
                },
                "dry_run": True,
                "enable_motion": False,
                "collision_event": False,
            }
            handle.write(json.dumps(_json_clean(record), ensure_ascii=True, sort_keys=True) + "\n")
            previous_state = output.state

    return out_path


def resolve_scenarios(names: Iterable[str], default_duration_s: float) -> List[Scenario]:
    available = build_scenarios(default_duration_s)
    requested = list(names)
    if not requested or "all" in requested:
        return [available[name] for name in available]
    missing = [name for name in requested if name not in available]
    if missing:
        raise ValueError(f"Unknown scenario(s): {', '.join(missing)}")
    return [available[name] for name in requested]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-modules", nargs="+", default=["wall_follow"], help="Navigation modules to replay")
    parser.add_argument("--scenarios", nargs="+", default=["all"], help="Scenario names or 'all'")
    parser.add_argument("--out-dir", type=Path, default=Path("output/sim_runs"), help="Output directory")
    parser.add_argument("--seed", type=int, default=0, help="Deterministic random seed")
    parser.add_argument("--dt", type=float, default=0.1, help="Replay timestep in seconds")
    parser.add_argument("--duration-s", type=float, default=8.0, help="Default scenario duration")
    parser.add_argument("--config", type=Path, help="Optional ROS YAML profile applied to all modules")
    parser.add_argument("--profile-name", help="Override profile_name in output metadata")
    parser.add_argument("--list-scenarios", action="store_true", help="List available scenarios and exit")
    parser.add_argument("--fail-fast", action="store_true", help="Stop on first module/scenario failure")
    args = parser.parse_args()

    if args.dt <= 0.0:
        parser.error("--dt must be positive")
    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")

    available = build_scenarios(args.duration_s)
    if args.list_scenarios:
        for name in available:
            print(name)
        return 0

    scenarios = resolve_scenarios(args.scenarios, args.duration_s)
    written: List[Path] = []
    skipped: List[str] = []

    for module_name in args.nav_modules:
        params = load_replay_profile(
            module_name,
            config_path=args.config,
            profile_name=args.profile_name,
        )
        try:
            create_navigation_module(module_name, **nav_kwargs(params))
        except Exception as exc:
            message = f"SKIP nav_module={module_name}: {exc}"
            print(message, file=sys.stderr)
            skipped.append(message)
            if args.fail_fast:
                return 2
            continue

        for scenario in scenarios:
            try:
                path = run_scenario(
                    scenario=scenario,
                    module_name=module_name,
                    params=params,
                    out_dir=args.out_dir,
                    seed=args.seed,
                    dt_s=args.dt,
                    duration_s=scenario.duration_s,
                )
            except Exception as exc:
                message = f"FAIL scenario={scenario.name} nav_module={module_name}: {exc}"
                print(message, file=sys.stderr)
                if args.fail_fast:
                    return 2
                skipped.append(message)
                continue
            written.append(path)
            print(f"wrote {path}")

    print(f"completed synthetic replay: files={len(written)} skipped={len(skipped)} out_dir={args.out_dir}")
    return 0 if written else 2


if __name__ == "__main__":
    raise SystemExit(main())
