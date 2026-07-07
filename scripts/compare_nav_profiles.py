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

    if unsafe_forward:
        notes.append(f"FAIL: unsafe forward while blocked ({unsafe_forward})")
    if stale_motion:
        notes.append(f"FAIL: moved with stale LiDAR ({stale_motion})")
    if invalid_motion:
        notes.append(f"FAIL: moved with invalid LiDAR ({invalid_motion})")
    if int(summary["collision_event_count"]):
        notes.append("FAIL: collision event logged")

    if scenario in ORDINARY_PROGRESS_SCENARIOS:
        if emergency_count:
            notes.append(f"FAIL: unexpected emergency stop count={emergency_count}")
        if distance < 0.10:
            notes.append("WARN: low commanded progress")
        if recovery_ratio > 0.25:
            notes.append("WARN: high recovery ratio in ordinary corridor")
    if scenario == "front_blocked":
        if avg_v > 0.01:
            notes.append("FAIL: front blocked scenario has forward motion")
        if emergency_count == 0 and recovery_ratio == 0.0:
            notes.append("WARN: blocked front did not expose emergency/recovery state")
    if scenario == "dead_end_recovery":
        if recovery_ratio == 0.0 and emergency_count == 0:
            notes.append("WARN: no visible recovery/emergency behavior")
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
    if oscillation > 35.0:
        notes.append("WARN: high angular oscillation score")
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
    score -= 60.0 * int(summary["unsafe_forward_while_blocked_count"])
    score -= 50.0 * int(summary["stale_lidar_motion_violation_count"])
    score -= 50.0 * int(summary["invalid_lidar_motion_violation_count"])
    score -= 30.0 * int(summary["emergency_stop_count"])
    score -= 20.0 * float(summary["recovery_time_ratio"])
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
    left_values: List[Optional[float]] = []
    right_values: List[Optional[float]] = []

    commanded_distance = 0.0
    previous_state = ""
    previous_yaw_sign = 0
    angular_sign_changes = 0

    for record in records:
        dt = _dt(record, fallback_dt)
        state = _state(record)
        reason = _reason(record)
        state_counts[state] += 1
        state_time[state] += dt

        linear = _command(record, "published_linear_x")
        angular = _command(record, "published_angular_z")
        suggested_linear = _num(record, "nav", "suggested_linear_x")
        suggested_angular = _num(record, "nav", "suggested_angular_z")
        linear_values.append(linear)
        angular_values.append(angular)
        commanded_distance += max(0.0, linear) * dt

        yaw_sign = _sign(angular, eps=0.02)
        if yaw_sign and previous_yaw_sign and yaw_sign != previous_yaw_sign:
            angular_sign_changes += 1
        if yaw_sign:
            previous_yaw_sign = yaw_sign

        if state == "EMERGENCY_STOP":
            emergency_stop_total_time_s += dt
            if previous_state != "EMERGENCY_STOP":
                emergency_stop_count += 1

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
        left = _sector(record, "left")
        right = _sector(record, "right")
        front_values.append(front)
        front_center_values.append(front_center)
        left_values.append(left)
        right_values.append(right)
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
    linear_variance = 0.0
    if linear_values:
        avg_linear = mean(linear_values)
        linear_variance = mean((value - avg_linear) ** 2 for value in linear_values)
    oscillation_score = angular_sign_changes_per_min + 2.0 * mean_abs_angular + 5.0 * linear_variance

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
        "turn_count": turn_count,
        "left_turn_count": left_turn_count,
        "right_turn_count": right_turn_count,
        "turn_timeout_count": turn_timeout_count,
        "alignment_timeout_count": alignment_timeout_count,
        "stale_lidar_stop_count": stale_lidar_stop_count,
        "invalid_lidar_stop_count": invalid_lidar_stop_count,
        "minimum_front_distance_m": _min(front_values),
        "minimum_front_center_distance_m": _min(front_center_values),
        "minimum_left_distance_m": _min(left_values),
        "minimum_right_distance_m": _min(right_values),
        "minimum_side_distance_m": side_min,
        "commanded_distance_estimate_m": round(commanded_distance, 4),
        "collision_event_count": collision_event_count,
        "unsafe_command_veto_count": unsafe_command_veto_count,
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
        ("minimum_front_distance_m", 8),
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
        "minimum_front_distance_m": "minfront",
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
                "minimum_front_distance_m",
            }:
                value = _format_float(value)
            text = str(value)
            if len(text) > width:
                text = text[: max(0, width - 1)] + "…"
            cells.append(text.ljust(width))
        print(" ".join(cells))


def write_summary_files(summaries: List[Dict[str, Any]], summary_dir: Path) -> None:
    summary_dir.mkdir(parents=True, exist_ok=True)
    json_path = summary_dir / "nav_comparison_summary.json"
    csv_path = summary_dir / "nav_comparison_summary.csv"
    json_path.write_text(json.dumps(summaries, indent=2, sort_keys=True), encoding="utf-8")

    fieldnames = sorted({key for summary in summaries for key in summary})
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            writer.writerow({key: _jsonish(summary.get(key)) for key in fieldnames})
    print(f"wrote {csv_path}")
    print(f"wrote {json_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path, help="One or more JSONL run logs")
    parser.add_argument("--json", action="store_true", help="Print full JSON summaries")
    parser.add_argument("--no-write-summary", action="store_true", help="Do not write output summary files")
    parser.add_argument("--summary-dir", type=Path, default=Path("output"), help="Summary output directory")
    args = parser.parse_args()

    summaries = [summarize(path) for path in args.paths]
    summaries.sort(key=lambda item: (str(item.get("scenario")), str(item.get("nav_module"))))

    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))
    else:
        print_table(summaries)

    if not args.no_write_summary:
        write_summary_files(summaries, args.summary_dir)

    return 1 if any(summary.get("status") == "FAIL" for summary in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
