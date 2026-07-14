#!/usr/bin/env python3
"""Compare reactive navigation JSONL runs across modules/profiles."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import sys
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from evaluate_nav_profiles import load_records, numeric  # noqa: E402


TURN_STATES = {"TURNING_LEFT", "TURNING_RIGHT"}
ALIGN_STATES = {"ALIGNING_AFTER_TURN", "SETTLING_AFTER_TURN"}
ORDINARY_PROGRESS_SCENARIOS = {
    "open_corridor",
    "narrow_corridor",
    "left_wall_close",
    "right_wall_close",
    "noisy_lidar_nan_inf",
    "spin_trap_open_space",
    "noisy_corridor_with_outliers",
    "oscillatory_corridor",
}
CORNER_SCENARIOS = {
    "front_left_corner_blocked",
    "front_right_corner_blocked",
    "corner_left_approach",
    "corner_right_approach",
    "narrow_left_turn",
    "narrow_right_turn",
    "noisy_corridor_with_outliers",
}
SIDE_RISK_SCENARIOS = {
    "left_wall_close",
    "right_wall_close",
    "asymmetric_corridor_left_close",
    "asymmetric_corridor_right_close",
    "wall_too_close_left",
    "wall_too_close_right",
}
RECOVERY_SCENARIOS = {
    "dead_end_recovery",
    "u_shape_dead_end",
}
SAFETY_STOP_SCENARIOS = {
    "front_blocked",
    "stale_lidar",
    "all_invalid_lidar",
}


def _step_records(records: Iterable[dict]) -> List[dict]:
    return [record for record in records if record.get("record_type", "step") != "metadata"]


def _metadata(records: Iterable[dict]) -> Dict[str, Any]:
    for record in records:
        if record.get("record_type") == "metadata":
            return record
    return {}


def _num(record: dict, *keys: str) -> Optional[float]:
    return numeric(record, *keys)


def _sector(record: dict, name: str) -> Optional[float]:
    direct = _num(record, "lidar", f"{name}_m")
    if direct is not None:
        return direct
    return _num(record, "lidar", "sector_distance_m", name)


def _command(record: dict, key: str) -> float:
    return _num(record, "command", key) or 0.0


def _dt(record: dict, fallback: float) -> float:
    value = _num(record, "dt_s")
    return fallback if value is None else max(0.0, value)


def _state(record: dict) -> str:
    return str(record.get("state") or "UNKNOWN")


def _reason(record: dict) -> str:
    return str(record.get("reason") or "")


def _sign(value: float, eps: float = 1e-3) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def _transition_count(records: List[dict], states: set[str]) -> int:
    count = 0
    previous = ""
    for record in records:
        state = _state(record)
        if state in states and previous != state:
            count += 1
        previous = state
    return count


def _min(values: Iterable[Optional[float]]) -> Optional[float]:
    finite = [value for value in values if value is not None and math.isfinite(value)]
    return min(finite) if finite else None


def _jsonish(value: Any) -> str:
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, sort_keys=True)
    return "" if value is None else str(value)


def _status_from_notes(notes: List[str]) -> str:
    if any(note.startswith("FAIL:") for note in notes):
        return "FAIL"
    if any(note.startswith("WARN:") for note in notes):
        return "WARN"
    return "PASS"


def _scenario_notes(summary: Dict[str, Any]) -> List[str]:
    notes: List[str] = []
    scenario = str(summary["scenario"])
    avg_v = float(summary["average_published_linear_speed_mps"])
    distance = float(summary["commanded_distance_estimate_m"])
    emergency_count = int(summary["emergency_stop_count"])
    recovery_ratio = float(summary["recovery_time_ratio"])
    oscillation = float(summary["oscillation_score"])
    turn_count = int(summary["turn_count"])
    unsafe_forward = int(summary["unsafe_forward_while_blocked_count"])
    stale_motion = int(summary["stale_lidar_motion_violation_count"])
    invalid_motion = int(summary["invalid_lidar_motion_violation_count"])
    qr_logged = int(summary["qr_logged_count"])
    corner_risk = int(summary["corner_risk_count"])
    side_risk = int(summary["side_risk_count"])
    spin_ratio = float(summary["spin_ratio"])
    yaw_saturation_ratio = float(summary["yaw_saturation_ratio"])
    low_progress_ratio = float(summary["low_progress_ratio"])
    recovery_loop_count = int(summary["recovery_loop_count"])
    recovery_timeout_count = int(summary["recovery_timeout_count"])

    if unsafe_forward:
        notes.append(f"FAIL: unsafe forward while blocked ({unsafe_forward})")
    if stale_motion:
        notes.append(f"FAIL: moved with stale LiDAR ({stale_motion})")
    if invalid_motion:
        notes.append(f"FAIL: moved with invalid LiDAR ({invalid_motion})")
    if int(summary["collision_event_count"]):
        notes.append("FAIL: collision event logged")
    if scenario in CORNER_SCENARIOS and corner_risk:
        notes.append(f"FAIL: corner-risk yaw count={corner_risk}")
    elif corner_risk:
        notes.append(f"WARN: corner-risk yaw count={corner_risk}")
    if scenario in SIDE_RISK_SCENARIOS and side_risk:
        notes.append(f"FAIL: side-risk yaw count={side_risk}")
    elif side_risk:
        notes.append(f"WARN: side-risk yaw count={side_risk}")

    if scenario in ORDINARY_PROGRESS_SCENARIOS:
        if emergency_count:
            notes.append(f"FAIL: unexpected emergency stop count={emergency_count}")
        if distance < 0.10:
            notes.append("WARN: low commanded progress")
        if low_progress_ratio > 0.85:
            notes.append("WARN: high low-progress ratio")
        if recovery_ratio > 0.25:
            notes.append("WARN: high recovery ratio in ordinary corridor")
    if scenario == "front_blocked":
        if avg_v > 0.01:
            notes.append("FAIL: front blocked scenario has forward motion")
        if emergency_count == 0 and recovery_ratio == 0.0:
            notes.append("WARN: blocked front did not expose emergency/recovery state")
    if scenario in RECOVERY_SCENARIOS:
        if recovery_ratio == 0.0 and emergency_count == 0:
            notes.append("WARN: no visible recovery/emergency behavior")
        if recovery_loop_count > 1:
            notes.append(f"WARN: recovery loop count={recovery_loop_count}")
        if recovery_timeout_count:
            notes.append(f"WARN: recovery timeout count={recovery_timeout_count}")
    if scenario == "stale_lidar" and int(summary["stale_lidar_stop_count"]) == 0:
        notes.append("FAIL: stale LiDAR stop was not logged")
    if scenario == "all_invalid_lidar" and int(summary["invalid_lidar_stop_count"]) == 0:
        notes.append("FAIL: invalid LiDAR stop was not logged")
    if scenario in {"left_sign_open", "right_sign_open", "repeated_sign_cooldown"}:
        if turn_count == 0:
            notes.append("FAIL: sign did not trigger a turn")
        if turn_count > 1:
            notes.append(f"FAIL: repeated sign turn count={turn_count}")
    if scenario in {"left_sign_blocked", "right_sign_blocked"} and turn_count > 0:
        notes.append(f"FAIL: blocked sign still triggered turn count={turn_count}")
    if scenario == "qr_visible":
        if qr_logged != 1:
            notes.append(f"FAIL: expected one QR log event, observed {qr_logged}")
        if int(summary["qr_duplicate_ignored_count"]) == 0:
            notes.append("WARN: duplicate QR suppression not observed")
    if scenario == "spin_trap_open_space" and spin_ratio > 0.20:
        notes.append(f"FAIL: spin ratio high ({spin_ratio:.2f})")
    elif spin_ratio > 0.35:
        notes.append(f"WARN: spin ratio high ({spin_ratio:.2f})")
    if oscillation > 35.0:
        notes.append("WARN: high angular oscillation score")
    if yaw_saturation_ratio > 0.60:
        notes.append(f"WARN: high yaw saturation ratio ({yaw_saturation_ratio:.2f})")
    if int(summary["turn_timeout_count"]):
        notes.append("WARN: turn timeout observed")
    if int(summary["alignment_timeout_count"]):
        notes.append("WARN: alignment timeout observed")

    return notes or ["PASS: scenario-specific checks satisfied"]


def _score(summary: Dict[str, Any], status: str) -> float:
    completed = 0.0 if status == "FAIL" else 1.0
    score = 100.0 * completed
    score += 20.0 * float(summary["commanded_distance_estimate_m"])
    score += 10.0 * int(summary["confirmed_sign_count"])
    score += 10.0 * int(summary["qr_logged_count"])
    score -= 100.0 * int(summary["collision_event_count"])
    score -= 150.0 * int(summary["corner_risk_count"])
    score -= 80.0 * int(summary["side_risk_count"])
    score -= 60.0 * int(summary["unsafe_forward_while_blocked_count"])
    score -= 50.0 * int(summary["stale_lidar_motion_violation_count"])
    score -= 50.0 * int(summary["invalid_lidar_motion_violation_count"])
    score -= 50.0 * int(summary["recovery_loop_count"])
    score -= 35.0 * int(summary["recovery_timeout_count"])
    score -= 30.0 * int(summary["emergency_stop_count"])
    score -= 40.0 * float(summary["spin_ratio"])
    score -= 20.0 * float(summary["recovery_time_ratio"])
    score -= 20.0 * float(summary["yaw_saturation_ratio"])
    score -= 15.0 * float(summary["angular_smoothness_cost"])
    score -= 10.0 * float(summary["oscillation_score"])
    score -= 10.0 * int(summary["turn_timeout_count"])
    score -= 10.0 * int(summary["alignment_timeout_count"])
    return round(score, 3)


def summarize(path: Path) -> Dict[str, Any]:
    all_records = load_records(path)
    metadata = _metadata(all_records)
    records = _step_records(all_records)
    if not records:
        return {
            "path": str(path),
            "scenario": metadata.get("scenario") or "UNKNOWN",
            "profile_name": metadata.get("profile_name"),
            "nav_module": metadata.get("nav_module"),
            "status": "FAIL",
            "notes": "FAIL: no step records",
            "scenario_score": -100.0,
        }

    fallback_dt = float(metadata.get("dt_s") or 0.1)
    config = metadata.get("config") if isinstance(metadata.get("config"), dict) else {}
    front_stop_distance = float(config.get("front_stop_distance") or 0.32)
    front_corner_avoid_distance = float(config.get("front_corner_avoid_distance") or 0.62)
    side_avoid_distance = float(config.get("side_avoid_distance") or 0.34)
    max_yaw = abs(float(config.get("max_yaw") or 0.65))
    spin_yaw_threshold = max(0.35, 0.60 * max_yaw)
    spin_linear_threshold = 0.025
    scenario = str(metadata.get("scenario") or records[-1].get("scenario") or "UNKNOWN")
    profile_name = str(metadata.get("profile_name") or records[-1].get("profile_name") or "unknown")
    nav_module = str(
        metadata.get("nav_module")
        or records[-1].get("nav", {}).get("module")
        or "unknown"
    )

    state_time: Dict[str, float] = defaultdict(float)
    state_counts = Counter()
    emergency_stop_count = 0
    emergency_stop_total_time_s = 0.0
    stale_lidar_stop_count = 0
    invalid_lidar_stop_count = 0
    stale_motion_violations = 0
    invalid_motion_violations = 0
    unsafe_forward_while_blocked = 0
    unsafe_command_veto_count = 0
    collision_event_count = 0
    corner_risk_count = 0
    front_left_risk_count = 0
    front_right_risk_count = 0
    side_risk_count = 0
    left_side_risk_count = 0
    right_side_risk_count = 0
    unsafe_yaw_veto_count = 0
    spin_tick_count = 0
    yaw_saturation_tick_count = 0
    low_progress_tick_count = 0
    active_motion_time_s = 0.0
    recovery_entry_count = 0
    recovery_loop_count = 0
    recovery_timeout_count = 0
    state_transition_count = 0
    confirmed_sign_count = 0
    qr_logged_count = 0
    qr_duplicate_ignored_count = 0
    turn_timeout_count = 0
    alignment_timeout_count = 0
    blocked_turn_suppression_count = 0
    cooldown_suppression_count = 0

    linear_values: List[float] = []
    angular_values: List[float] = []
    front_values: List[Optional[float]] = []
    front_center_values: List[Optional[float]] = []
    front_left_values: List[Optional[float]] = []
    front_right_values: List[Optional[float]] = []
    left_values: List[Optional[float]] = []
    right_values: List[Optional[float]] = []

    commanded_distance = 0.0
    previous_state = ""
    previous_yaw_sign = 0
    previous_angular: Optional[float] = None
    angular_sign_changes = 0
    angular_smoothness_cost = 0.0
    last_recovery_exit_seen = False

    for record in records:
        dt = _dt(record, fallback_dt)
        state = _state(record)
        reason = _reason(record)
        state_counts[state] += 1
        state_time[state] += dt
        if previous_state and state != previous_state:
            state_transition_count += 1

        linear = _command(record, "published_linear_x")
        angular = _command(record, "published_angular_z")
        suggested_linear = _num(record, "nav", "suggested_linear_x")
        suggested_angular = _num(record, "nav", "suggested_angular_z")
        linear_values.append(linear)
        angular_values.append(angular)
        commanded_distance += max(0.0, linear) * dt
        if abs(linear) > 0.01 or abs(angular) > 0.02:
            active_motion_time_s += dt
        if abs(linear) < 0.01 and state not in {"EMERGENCY_STOP", "QR_SCAN", "MANUAL_STOP"}:
            low_progress_tick_count += 1
        if abs(angular) >= 0.90 * max_yaw and max_yaw > 0.0:
            yaw_saturation_tick_count += 1
        if previous_angular is not None:
            angular_smoothness_cost += abs(angular - previous_angular)
        previous_angular = angular

        yaw_sign = _sign(angular, eps=0.02)
        if yaw_sign and previous_yaw_sign and yaw_sign != previous_yaw_sign:
            angular_sign_changes += 1
        if yaw_sign:
            previous_yaw_sign = yaw_sign

        if state == "EMERGENCY_STOP":
            emergency_stop_total_time_s += dt
            if previous_state != "EMERGENCY_STOP":
                emergency_stop_count += 1
        if "RECOVERY" in state and "RECOVERY" not in previous_state:
            recovery_entry_count += 1
            if last_recovery_exit_seen:
                recovery_loop_count += 1
            last_recovery_exit_seen = False
        elif "RECOVERY" not in state and "RECOVERY" in previous_state:
            last_recovery_exit_seen = True
        if "RECOVERY" in state and "TIMEOUT" in reason:
            recovery_timeout_count += 1

        lidar_fresh = bool(record.get("freshness", {}).get("lidar_fresh", record.get("lidar", {}).get("fresh", True)))
        lidar_valid_count = int(record.get("lidar", {}).get("valid_count") or 0)
        moving = abs(linear) > 0.01 or abs(angular) > 0.02
        if not lidar_fresh:
            if state == "EMERGENCY_STOP" or "STALE" in reason:
                stale_lidar_stop_count += 1
            if moving:
                stale_motion_violations += 1
        if lidar_valid_count == 0:
            if state == "EMERGENCY_STOP" or "NO_VALID" in reason or "INVALID" in reason:
                invalid_lidar_stop_count += 1
            if moving:
                invalid_motion_violations += 1

        front = _sector(record, "front")
        front_center = _sector(record, "front_center")
        front_left = _sector(record, "front_left")
        front_right = _sector(record, "front_right")
        left = _sector(record, "left")
        right = _sector(record, "right")
        front_values.append(front)
        front_center_values.append(front_center)
        front_left_values.append(front_left)
        front_right_values.append(front_right)
        left_values.append(left)
        right_values.append(right)
        front_left_risky = front_left is not None and front_left < front_corner_avoid_distance
        front_right_risky = front_right is not None and front_right < front_corner_avoid_distance
        left_side_risky = left is not None and left < side_avoid_distance
        right_side_risky = right is not None and right < side_avoid_distance
        if front_left_risky:
            front_left_risk_count += 1
        if front_right_risky:
            front_right_risk_count += 1
        unsafe_left_yaw = front_left_risky and angular > 0.02
        unsafe_right_yaw = front_right_risky and angular < -0.02
        if unsafe_left_yaw or unsafe_right_yaw:
            corner_risk_count += 1
        if left_side_risky and angular > 0.02:
            left_side_risk_count += 1
            side_risk_count += 1
        if right_side_risky and angular < -0.02:
            right_side_risk_count += 1
            side_risk_count += 1
        if state not in TURN_STATES and state not in ALIGN_STATES:
            if abs(angular) > spin_yaw_threshold and abs(linear) < spin_linear_threshold:
                spin_tick_count += 1
        blocked_front = (
            (front is not None and front < front_stop_distance)
            or (front_center is not None and front_center < front_stop_distance)
        )
        if blocked_front and linear > 0.01:
            unsafe_forward_while_blocked += 1

        if suggested_linear is not None or suggested_angular is not None:
            suggested_linear = suggested_linear or 0.0
            suggested_angular = suggested_angular or 0.0
            if abs(suggested_linear - linear) > 1e-3 or abs(suggested_angular - angular) > 1e-3:
                unsafe_command_veto_count += 1
            if abs(suggested_angular - angular) > 1e-3:
                unsafe_yaw_veto_count += 1

        if record.get("collision_event"):
            collision_event_count += 1
        if state in TURN_STATES and previous_state not in TURN_STATES:
            confirmed_sign_count += 1
        if "TIMEOUT" in reason and state in TURN_STATES:
            turn_timeout_count += 1
        turn_completed = str(record.get("turn", {}).get("turn_completed_reason") or "")
        if "TURN_TIMEOUT" in turn_completed:
            turn_timeout_count += 1
        if "ALIGNMENT_TIMEOUT" in reason or "ALIGNMENT_TIMEOUT" in turn_completed:
            alignment_timeout_count += 1
        if "BLOCKED" in reason and state == "SIGN_CANDIDATE":
            blocked_turn_suppression_count += 1
        if "COOLDOWN" in reason.upper():
            cooldown_suppression_count += 1
        if bool(record.get("qr", {}).get("logged")):
            qr_logged_count += 1
        if bool(record.get("qr", {}).get("duplicate")):
            qr_duplicate_ignored_count += 1

        previous_state = state

    total_runtime_s = sum(_dt(record, fallback_dt) for record in records)
    turn_count = _transition_count(records, TURN_STATES)
    left_turn_count = _transition_count(records, {"TURNING_LEFT"})
    right_turn_count = _transition_count(records, {"TURNING_RIGHT"})
    recovery_time_s = sum(value for state, value in state_time.items() if "RECOVERY" in state)
    recovery_time_ratio = recovery_time_s / total_runtime_s if total_runtime_s > 0.0 else 0.0
    mean_abs_angular = mean(abs(value) for value in angular_values) if angular_values else 0.0
    angular_sign_changes_per_min = (
        angular_sign_changes / (total_runtime_s / 60.0) if total_runtime_s > 0.0 else 0.0
    )
    state_transition_count_per_min = (
        state_transition_count / (total_runtime_s / 60.0) if total_runtime_s > 0.0 else 0.0
    )
    linear_variance = 0.0
    if linear_values:
        avg_linear = mean(linear_values)
        linear_variance = mean((value - avg_linear) ** 2 for value in linear_values)
    oscillation_score = angular_sign_changes_per_min + 2.0 * mean_abs_angular + 5.0 * linear_variance
    total_ticks = len(records)
    spin_ratio = spin_tick_count / total_ticks if total_ticks else 0.0
    yaw_saturation_ratio = yaw_saturation_tick_count / total_ticks if total_ticks else 0.0
    low_progress_ratio = low_progress_tick_count / total_ticks if total_ticks else 0.0
    angular_smoothness_mean = angular_smoothness_cost / max(1, len(angular_values) - 1)

    side_min = _min([_min(left_values), _min(right_values)])
    summary: Dict[str, Any] = {
        "path": str(path),
        "scenario": scenario,
        "profile_name": profile_name,
        "nav_module": nav_module,
        "total_runtime_s": round(total_runtime_s, 3),
        "time_per_state": dict(sorted((state, round(value, 3)) for state, value in state_time.items())),
        "state_counts": dict(state_counts),
        "emergency_stop_count": emergency_stop_count,
        "emergency_stop_total_time_s": round(emergency_stop_total_time_s, 3),
        "recovery_time_ratio": round(recovery_time_ratio, 4),
        "average_published_linear_speed_mps": round(mean(linear_values) if linear_values else 0.0, 4),
        "mean_abs_angular_speed_radps": round(mean_abs_angular, 4),
        "angular_sign_changes_per_min": round(angular_sign_changes_per_min, 3),
        "oscillation_score": round(oscillation_score, 3),
        "yaw_saturation_ratio": round(yaw_saturation_ratio, 4),
        "angular_smoothness_cost": round(angular_smoothness_mean, 4),
        "spin_ratio": round(spin_ratio, 4),
        "low_progress_ratio": round(low_progress_ratio, 4),
        "active_motion_time_s": round(active_motion_time_s, 3),
        "state_transition_count_per_min": round(state_transition_count_per_min, 3),
        "turn_count": turn_count,
        "left_turn_count": left_turn_count,
        "right_turn_count": right_turn_count,
        "turn_timeout_count": turn_timeout_count,
        "alignment_timeout_count": alignment_timeout_count,
        "recovery_entry_count": recovery_entry_count,
        "recovery_loop_count": recovery_loop_count,
        "recovery_timeout_count": recovery_timeout_count,
        "stale_lidar_stop_count": stale_lidar_stop_count,
        "invalid_lidar_stop_count": invalid_lidar_stop_count,
        "minimum_front_distance_m": _min(front_values),
        "minimum_front_center_distance_m": _min(front_center_values),
        "minimum_front_left_distance_m": _min(front_left_values),
        "minimum_front_right_distance_m": _min(front_right_values),
        "minimum_left_distance_m": _min(left_values),
        "minimum_right_distance_m": _min(right_values),
        "minimum_side_distance_m": side_min,
        "commanded_distance_estimate_m": round(commanded_distance, 4),
        "collision_event_count": collision_event_count,
        "corner_risk_count": corner_risk_count,
        "front_left_risk_count": front_left_risk_count,
        "front_right_risk_count": front_right_risk_count,
        "side_risk_count": side_risk_count,
        "left_side_risk_count": left_side_risk_count,
        "right_side_risk_count": right_side_risk_count,
        "unsafe_command_veto_count": unsafe_command_veto_count,
        "unsafe_yaw_veto_count": unsafe_yaw_veto_count,
        "unsafe_forward_while_blocked_count": unsafe_forward_while_blocked,
        "stale_lidar_motion_violation_count": stale_motion_violations,
        "invalid_lidar_motion_violation_count": invalid_motion_violations,
        "confirmed_sign_count": confirmed_sign_count,
        "blocked_turn_suppression_count": blocked_turn_suppression_count,
        "cooldown_suppression_count": cooldown_suppression_count,
        "qr_logged_count": qr_logged_count,
        "qr_duplicate_ignored_count": qr_duplicate_ignored_count,
    }
    notes = _scenario_notes(summary)
    status = _status_from_notes(notes)
    summary["status"] = status
    summary["scenario_score"] = _score(summary, status)
    summary["notes"] = "; ".join(notes)
    return summary


def _format_float(value: Any, digits: int = 3) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float)):
        return f"{float(value):.{digits}f}"
    return str(value)


def print_table(summaries: List[Dict[str, Any]]) -> None:
    columns = [
        ("scenario", 22),
        ("profile_name", 18),
        ("nav_module", 10),
        ("total_runtime_s", 8),
        ("scenario_score", 8),
        ("status", 6),
        ("emergency_stop_count", 5),
        ("recovery_time_ratio", 7),
        ("average_published_linear_speed_mps", 7),
        ("mean_abs_angular_speed_radps", 7),
        ("angular_sign_changes_per_min", 7),
        ("corner_risk_count", 6),
        ("side_risk_count", 5),
        ("spin_ratio", 6),
        ("yaw_saturation_ratio", 6),
        ("minimum_front_distance_m", 8),
        ("minimum_front_left_distance_m", 8),
        ("minimum_front_right_distance_m", 8),
        ("turn_count", 5),
        ("stale_lidar_stop_count", 5),
        ("unsafe_command_veto_count", 5),
        ("notes", 36),
    ]
    header_labels = {
        "profile_name": "profile",
        "nav_module": "module",
        "total_runtime_s": "runtime",
        "scenario_score": "score",
        "emergency_stop_count": "emerg",
        "recovery_time_ratio": "recov",
        "average_published_linear_speed_mps": "avg_v",
        "mean_abs_angular_speed_radps": "abs_w",
        "angular_sign_changes_per_min": "osc/min",
        "corner_risk_count": "corner",
        "side_risk_count": "side",
        "spin_ratio": "spin",
        "yaw_saturation_ratio": "sat",
        "minimum_front_distance_m": "minfront",
        "minimum_front_left_distance_m": "minFL",
        "minimum_front_right_distance_m": "minFR",
        "turn_count": "turns",
        "stale_lidar_stop_count": "stale",
        "unsafe_command_veto_count": "veto",
    }
    print(
        " ".join(
            (header_labels.get(key, key)[:width]).ljust(width)
            for key, width in columns
        )
    )
    print(" ".join("-" * width for _key, width in columns))
    for summary in summaries:
        cells = []
        for key, width in columns:
            value = summary.get(key)
            if key in {
                "total_runtime_s",
                "scenario_score",
                "recovery_time_ratio",
                "average_published_linear_speed_mps",
                "mean_abs_angular_speed_radps",
                "angular_sign_changes_per_min",
                "spin_ratio",
                "yaw_saturation_ratio",
                "minimum_front_distance_m",
                "minimum_front_left_distance_m",
                "minimum_front_right_distance_m",
            }:
                value = _format_float(value)
            text = str(value)
            if len(text) > width:
                text = text[: max(0, width - 1)] + "…"
            cells.append(text.ljust(width))
        print(" ".join(cells))


def _aggregate_by_module(summaries: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for summary in summaries:
        grouped[str(summary.get("nav_module"))].append(summary)
    rows = []
    for module, items in sorted(grouped.items()):
        rows.append(
            {
                "nav_module": module,
                "runs": len(items),
                "pass": sum(item.get("status") == "PASS" for item in items),
                "warn": sum(item.get("status") == "WARN" for item in items),
                "fail": sum(item.get("status") == "FAIL" for item in items),
                "avg_score": round(mean(float(item.get("scenario_score") or 0.0) for item in items), 3),
                "corner_risk_count": sum(int(item.get("corner_risk_count") or 0) for item in items),
                "side_risk_count": sum(int(item.get("side_risk_count") or 0) for item in items),
                "avg_spin_ratio": round(mean(float(item.get("spin_ratio") or 0.0) for item in items), 4),
                "avg_oscillation_score": round(mean(float(item.get("oscillation_score") or 0.0) for item in items), 3),
            }
        )
    return rows


def _load_baseline_csv(path: Optional[Path]) -> Dict[tuple[str, str], Dict[str, str]]:
    if path is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        return {
            (row.get("scenario", ""), row.get("nav_module", "")): row
            for row in csv.DictReader(handle)
        }


def _write_markdown_summary(
    summaries: List[Dict[str, Any]],
    md_path: Path,
    baseline_rows: Optional[Dict[tuple[str, str], Dict[str, str]]] = None,
) -> None:
    md_path.parent.mkdir(parents=True, exist_ok=True)
    aggregate = _aggregate_by_module(summaries)
    lines = [
        "# Navigation Comparison Summary",
        "",
        "Offline synthetic validation only. These results do not prove physical robot readiness.",
        "",
        "## Aggregate By Module",
        "",
        "| module | runs | pass | warn | fail | avg score | corner risk | side risk | avg spin | avg oscillation |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in aggregate:
        lines.append(
            "| {nav_module} | {runs} | {pass} | {warn} | {fail} | {avg_score:.3f} | "
            "{corner_risk_count} | {side_risk_count} | {avg_spin_ratio:.4f} | "
            "{avg_oscillation_score:.3f} |".format(**row)
        )

    lines.extend(
        [
            "",
            "## Runs Requiring Attention",
            "",
            "| scenario | module | status | score | notes |",
            "| --- | --- | --- | ---: | --- |",
        ]
    )
    attention = [item for item in summaries if item.get("status") != "PASS"]
    for item in attention or []:
        lines.append(
            f"| {item.get('scenario')} | {item.get('nav_module')} | {item.get('status')} | "
            f"{float(item.get('scenario_score') or 0.0):.3f} | {item.get('notes')} |"
        )
    if not attention:
        lines.append("| - | - | PASS | - | No WARN/FAIL rows |")

    if baseline_rows:
        lines.extend(
            [
                "",
                "## Score Delta vs Baseline",
                "",
                "| scenario | module | score delta | status now | status baseline |",
                "| --- | --- | ---: | --- | --- |",
            ]
        )
        for item in summaries:
            key = (str(item.get("scenario")), str(item.get("nav_module")))
            baseline = baseline_rows.get(key)
            if not baseline:
                continue
            try:
                delta = float(item.get("scenario_score") or 0.0) - float(baseline.get("scenario_score") or 0.0)
            except ValueError:
                continue
            lines.append(
                f"| {key[0]} | {key[1]} | {delta:.3f} | {item.get('status')} | {baseline.get('status')} |"
            )

    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {md_path}")


def write_summary_files(
    summaries: List[Dict[str, Any]],
    summary_dir: Path,
    *,
    summary_csv: Optional[Path] = None,
    summary_json: Optional[Path] = None,
    summary_md: Optional[Path] = None,
    baseline_csv: Optional[Path] = None,
) -> None:
    summary_dir.mkdir(parents=True, exist_ok=True)
    json_path = summary_json or summary_dir / "nav_comparison_summary.json"
    csv_path = summary_csv or summary_dir / "nav_comparison_summary.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(summaries, indent=2, sort_keys=True), encoding="utf-8")

    fieldnames = sorted({key for summary in summaries for key in summary})
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            writer.writerow({key: _jsonish(summary.get(key)) for key in fieldnames})
    print(f"wrote {csv_path}")
    print(f"wrote {json_path}")
    if summary_md is not None:
        _write_markdown_summary(summaries, summary_md, _load_baseline_csv(baseline_csv))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path, help="One or more JSONL run logs")
    parser.add_argument("--json", action="store_true", help="Print full JSON summaries")
    parser.add_argument("--no-write-summary", action="store_true", help="Do not write output summary files")
    parser.add_argument("--summary-dir", type=Path, default=Path("output"), help="Summary output directory")
    parser.add_argument("--summary-csv", type=Path, help="Write summary CSV to this path")
    parser.add_argument("--summary-json", type=Path, help="Write summary JSON to this path")
    parser.add_argument("--summary-md", type=Path, help="Write human-readable markdown summary to this path")
    parser.add_argument("--baseline", type=Path, help="Optional baseline summary CSV for markdown deltas")
    args = parser.parse_args()

    summaries = [summarize(path) for path in args.paths]
    summaries.sort(key=lambda item: (str(item.get("scenario")), str(item.get("nav_module"))))

    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))
    else:
        print_table(summaries)

    if not args.no_write_summary:
        write_summary_files(
            summaries,
            args.summary_dir,
            summary_csv=args.summary_csv,
            summary_json=args.summary_json,
            summary_md=args.summary_md,
            baseline_csv=args.baseline,
        )

    return 1 if any(summary.get("status") == "FAIL" for summary in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
