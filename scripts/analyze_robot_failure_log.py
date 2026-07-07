#!/usr/bin/env python3
"""Detect navigation failure patterns in robot/debug JSONL logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def load_records(path: Path) -> List[dict]:
    records = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(payload, dict) and payload.get("record_type") != "metadata":
                    records.append(payload)
    except OSError:
        return []
    return records


def nested(record: dict, *keys: str) -> Any:
    value: Any = record
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def number(record: dict, *keys: str) -> Optional[float]:
    value = nested(record, *keys)
    if isinstance(value, (int, float)):
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None
    return None


def sector(record: dict, name: str) -> Optional[float]:
    for path in (
        ("lidar", f"{name}_m"),
        ("lidar", "sector_distance_m", name),
        ("lidar", name),
    ):
        value = number(record, *path)
        if value is not None:
            return value
    return None


def state(record: dict) -> str:
    return str(record.get("state") or "UNKNOWN")


def command(record: dict, name: str) -> float:
    return number(record, "command", f"published_{name}") or number(record, "command", f"requested_{name}") or 0.0


def time_s(record: dict, index: int, dt: float) -> float:
    value = record.get("time_s")
    if isinstance(value, (int, float)):
        return float(value)
    value = record.get("timestamp")
    if isinstance(value, (int, float)):
        return float(value)
    return index * dt


def add_interval(intervals: List[Dict[str, Any]], path: Path, records: List[dict], index: int, kind: str, detail: str) -> None:
    record = records[index]
    intervals.append(
        {
            "path": str(path),
            "index": index,
            "time_s": round(time_s(record, index, number(record, "dt_s") or 0.1), 3),
            "kind": kind,
            "state": state(record),
            "detail": detail,
            "linear_x": command(record, "linear_x"),
            "angular_z": command(record, "angular_z"),
            "front": sector(record, "front"),
            "front_left": sector(record, "front_left"),
            "front_right": sector(record, "front_right"),
            "left": sector(record, "left"),
            "right": sector(record, "right"),
        }
    )


def analyze_file(path: Path) -> List[Dict[str, Any]]:
    records = load_records(path)
    intervals: List[Dict[str, Any]] = []
    if not records:
        return intervals

    corner_threshold = 0.62
    side_threshold = 0.24
    spin_yaw_threshold = 0.38
    spin_linear_threshold = 0.03
    previous_yaw_sign = 0
    sign_changes = 0
    recovery_entries = 0
    previous_state = ""

    for index, record in enumerate(records):
        linear = command(record, "linear_x")
        yaw = command(record, "angular_z")
        current_state = state(record)
        front_left = sector(record, "front_left")
        front_right = sector(record, "front_right")
        left = sector(record, "left")
        right = sector(record, "right")

        if current_state == "RECOVERY" and previous_state != "RECOVERY":
            recovery_entries += 1
            add_interval(intervals, path, records, index, "recovery_entry", "entered RECOVERY")
        if current_state == "EMERGENCY_STOP" and previous_state != "EMERGENCY_STOP":
            add_interval(intervals, path, records, index, "emergency_burst", str(record.get("reason") or "emergency"))

        if current_state not in {"TURNING_LEFT", "TURNING_RIGHT", "ALIGNING_AFTER_TURN"}:
            if abs(yaw) > spin_yaw_threshold and abs(linear) < spin_linear_threshold:
                add_interval(intervals, path, records, index, "spin", "high yaw with low linear speed")

        if front_left is not None and front_left < corner_threshold and yaw > 0.02:
            add_interval(intervals, path, records, index, "corner_risk", "front_left close with positive yaw")
        if front_right is not None and front_right < corner_threshold and yaw < -0.02:
            add_interval(intervals, path, records, index, "corner_risk", "front_right close with negative yaw")
        if left is not None and left < side_threshold and yaw > 0.02:
            add_interval(intervals, path, records, index, "side_scrape_risk", "left side close with positive yaw")
        if right is not None and right < side_threshold and yaw < -0.02:
            add_interval(intervals, path, records, index, "side_scrape_risk", "right side close with negative yaw")

        yaw_sign = 1 if yaw > 0.03 else -1 if yaw < -0.03 else 0
        if yaw_sign and previous_yaw_sign and yaw_sign != previous_yaw_sign:
            sign_changes += 1
            add_interval(intervals, path, records, index, "oscillation", "angular command sign changed")
        if yaw_sign:
            previous_yaw_sign = yaw_sign
        previous_state = current_state

    if recovery_entries >= 3:
        intervals.append(
            {
                "path": str(path),
                "index": -1,
                "time_s": None,
                "kind": "recovery_loop",
                "state": "SUMMARY",
                "detail": f"{recovery_entries} recovery entries",
            }
        )
    if sign_changes >= 10:
        intervals.append(
            {
                "path": str(path),
                "index": -1,
                "time_s": None,
                "kind": "oscillation_summary",
                "state": "SUMMARY",
                "detail": f"{sign_changes} angular sign changes",
            }
        )
    return intervals


def suggested_scenarios(intervals: Iterable[Dict[str, Any]]) -> List[str]:
    kinds = {str(interval.get("kind")) for interval in intervals}
    suggestions = []
    if "corner_risk" in kinds:
        suggestions.append("front_left_corner_blocked / front_right_corner_blocked")
        suggestions.append("corner_left_approach / corner_right_approach")
    if "spin" in kinds:
        suggestions.append("spin_trap_open_space")
    if "recovery_loop" in kinds or "recovery_entry" in kinds:
        suggestions.append("u_shape_dead_end")
    if "side_scrape_risk" in kinds:
        suggestions.append("wall_too_close_left / wall_too_close_right")
    if "oscillation" in kinds or "oscillation_summary" in kinds:
        suggestions.append("oscillatory_corridor")
    return sorted(set(suggestions))


def write_outputs(intervals: List[Dict[str, Any]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "failure_intervals.csv"
    md_path = out_dir / "failure_summary.md"
    scenarios_path = out_dir / "suggested_scenarios.md"

    fieldnames = sorted({key for interval in intervals for key in interval}) or ["path", "kind", "detail"]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(intervals)

    counts: Dict[str, int] = {}
    for interval in intervals:
        counts[str(interval.get("kind"))] = counts.get(str(interval.get("kind")), 0) + 1
    lines = [
        "# Robot Failure Analysis",
        "",
        "Offline log analysis only. This does not validate a fix on the robot.",
        "",
        "## Counts",
        "",
    ]
    if counts:
        for kind, count in sorted(counts.items()):
            lines.append(f"- `{kind}`: {count}")
    else:
        lines.append("- No failure intervals detected or no useful records found.")
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    suggestions = suggested_scenarios(intervals)
    scenario_lines = ["# Suggested Synthetic Scenarios", ""]
    if suggestions:
        scenario_lines.extend(f"- {item}" for item in suggestions)
    else:
        scenario_lines.append("- No scenario recommendations from supplied logs.")
    scenarios_path.write_text("\n".join(scenario_lines) + "\n", encoding="utf-8")

    print(f"wrote {csv_path}")
    print(f"wrote {md_path}")
    print(f"wrote {scenarios_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("output/robot_failure_analysis"))
    args = parser.parse_args()

    intervals: List[Dict[str, Any]] = []
    useful_files = 0
    for path in args.paths:
        file_intervals = analyze_file(path)
        if load_records(path):
            useful_files += 1
        intervals.extend(file_intervals)
    write_outputs(intervals, args.out_dir)
    print(f"analyzed_files={len(args.paths)} useful_files={useful_files} intervals={len(intervals)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
