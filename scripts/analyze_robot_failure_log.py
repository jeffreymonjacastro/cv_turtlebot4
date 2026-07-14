#!/usr/bin/env python3
"""Detect navigation failure intervals in robot/debug JSONL logs.

This is an offline-only analyzer. It reads persistent navigation logs, groups
failure-looking ticks into intervals, and writes artifacts that can drive
regression replay and ablation work. It never publishes /cmd_vel.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional


DEFAULT_PATTERNS = (
    "output/reactive_nav_debug*.jsonl",
    "output/robot_runs/reactive_nav_debug*.jsonl",
)
TURN_STATES = {"TURNING_LEFT", "TURNING_RIGHT", "ALIGNING_AFTER_TURN", "SETTLING_AFTER_TURN"}


def load_records(path: Path) -> List[dict]:
    records: List[dict] = []
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
    return (
        number(record, "command", f"published_{name}")
        or number(record, "command", f"requested_{name}")
        or number(record, "nav", f"suggested_{name}")
        or 0.0
    )


def time_s(record: dict, index: int, dt: float) -> float:
    value = record.get("time_s")
    if isinstance(value, (int, float)):
        return float(value)
    value = record.get("timestamp")
    if isinstance(value, (int, float)):
        return float(value)
    return index * dt


def _run_id(path: Path) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in path.stem)


def _sign(value: float, eps: float = 0.03) -> int:
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def _min(values: Iterable[Optional[float]]) -> Optional[float]:
    finite = [value for value in values if value is not None and math.isfinite(value)]
    return min(finite) if finite else None


def _mean(values: Iterable[float]) -> float:
    items = list(values)
    return mean(items) if items else 0.0


def _record_flags(
    records: List[dict],
    *,
    dt: float,
    front_corner_avoid_distance: float,
    side_avoid_distance: float,
    max_yaw: float,
    spin_yaw_threshold: float,
    spin_linear_threshold: float,
) -> List[Dict[str, bool]]:
    flags: List[Dict[str, bool]] = []
    yaw_sign_window: List[int] = []
    state_window: List[str] = []
    previous_yaw_sign = 0
    previous_state = ""

    for index, record in enumerate(records):
        yaw = command(record, "angular_z")
        linear = command(record, "linear_x")
        current_state = state(record)
        front_left = sector(record, "front_left")
        front_right = sector(record, "front_right")
        left = sector(record, "left")
        right = sector(record, "right")

        front_left_risky = front_left is not None and front_left < front_corner_avoid_distance
        front_right_risky = front_right is not None and front_right < front_corner_avoid_distance
        left_risky = left is not None and left < side_avoid_distance
        right_risky = right is not None and right < side_avoid_distance
        yaw_sign = _sign(yaw)

        yaw_sign_window.append(yaw_sign)
        state_window.append(current_state)
        max_window = max(3, int(round(1.5 / dt)))
        yaw_sign_window = yaw_sign_window[-max_window:]
        state_window = state_window[-max_window:]
        nonzero_signs = [item for item in yaw_sign_window if item]
        sign_changes = sum(
            1
            for left_sign, right_sign in zip(nonzero_signs, nonzero_signs[1:])
            if left_sign != right_sign
        )
        state_changes = sum(
            1
            for left_state, right_state in zip(state_window, state_window[1:])
            if left_state != right_state
        )

        flags.append(
            {
                "corner_risk": (front_left_risky and yaw > 0.02) or (front_right_risky and yaw < -0.02),
                "side_scrape_risk": (left_risky and yaw > 0.02) or (right_risky and yaw < -0.02),
                "spin": (
                    current_state not in TURN_STATES
                    and abs(yaw) >= spin_yaw_threshold
                    and abs(linear) <= spin_linear_threshold
                ),
                "oscillation": sign_changes >= 3,
                "yaw_saturation": abs(yaw) >= 0.90 * max_yaw if max_yaw > 0.0 else False,
                "recovery_loop": current_state == "RECOVERY"
                and (
                    previous_state == "RECOVERY"
                    or any(item == "RECOVERY" for item in state_window[:-1])
                ),
                "emergency_burst": current_state == "EMERGENCY_STOP"
                and (previous_state == "EMERGENCY_STOP" or "EMERGENCY" in str(record.get("reason") or "")),
                "state_flapping": state_changes >= 4,
            }
        )
        if yaw_sign:
            previous_yaw_sign = yaw_sign
        previous_state = current_state

    return flags


def _interval_summary(
    *,
    path: Path,
    failure_type: str,
    records: List[dict],
    indices: List[int],
    dt: float,
) -> Dict[str, Any]:
    selected = [records[index] for index in indices]
    start_index = indices[0]
    end_index = indices[-1]
    start_time = time_s(records[start_index], start_index, dt)
    end_time = time_s(records[end_index], end_index, dt) + dt
    front_left_values = [sector(record, "front_left") for record in selected]
    front_right_values = [sector(record, "front_right") for record in selected]
    left_values = [sector(record, "left") for record in selected]
    right_values = [sector(record, "right") for record in selected]
    angular_values = [command(record, "angular_z") for record in selected]
    linear_values = [command(record, "linear_x") for record in selected]
    state_counts = Counter(state(record) for record in selected)

    if failure_type == "corner_risk":
        representative_index = min(
            indices,
            key=lambda i: min(
                value
                for value in (
                    sector(records[i], "front_left"),
                    sector(records[i], "front_right"),
                )
                if value is not None
            ),
        )
        notes = "turning toward a close front corner"
    elif failure_type == "side_scrape_risk":
        representative_index = min(
            indices,
            key=lambda i: min(
                value
                for value in (sector(records[i], "left"), sector(records[i], "right"))
                if value is not None
            ),
        )
        notes = "turning toward a close side wall"
    elif failure_type == "oscillation":
        representative_index = max(indices, key=lambda i: abs(command(records[i], "angular_z")))
        notes = "frequent angular sign changes"
    else:
        representative_index = max(indices, key=lambda i: abs(command(records[i], "angular_z")))
        notes = failure_type.replace("_", " ")

    return {
        "run_id": _run_id(path),
        "source_path": str(path),
        "failure_type": failure_type,
        "start_index": start_index,
        "end_index": end_index,
        "start_time": round(start_time, 3),
        "end_time": round(end_time, 3),
        "duration_s": round(max(dt, end_time - start_time), 3),
        "record_count": len(indices),
        "state_counts": dict(state_counts),
        "min_front_left_m": _min(front_left_values),
        "min_front_right_m": _min(front_right_values),
        "min_left_m": _min(left_values),
        "min_right_m": _min(right_values),
        "mean_linear_x": round(_mean(linear_values), 4),
        "mean_angular_z": round(_mean(angular_values), 4),
        "max_abs_angular_z": round(max((abs(value) for value in angular_values), default=0.0), 4),
        "representative_record_index": representative_index,
        "notes": notes,
    }


def _group_intervals(
    path: Path,
    records: List[dict],
    flags: List[Dict[str, bool]],
    *,
    dt: float,
) -> List[Dict[str, Any]]:
    intervals: List[Dict[str, Any]] = []
    failure_types = [
        "corner_risk",
        "side_scrape_risk",
        "spin",
        "oscillation",
        "yaw_saturation",
        "recovery_loop",
        "emergency_burst",
        "state_flapping",
    ]
    for failure_type in failure_types:
        active: List[int] = []
        for index, flag_set in enumerate(flags):
            if flag_set.get(failure_type):
                active.append(index)
                continue
            if active:
                intervals.append(
                    _interval_summary(path=path, failure_type=failure_type, records=records, indices=active, dt=dt)
                )
                active = []
        if active:
            intervals.append(
                _interval_summary(path=path, failure_type=failure_type, records=records, indices=active, dt=dt)
            )
    return intervals


def analyze_file(
    path: Path,
    *,
    dt: float,
    front_corner_avoid_distance: float,
    side_avoid_distance: float,
    max_yaw: float,
    spin_yaw_threshold: float,
    spin_linear_threshold: float,
) -> tuple[List[Dict[str, Any]], Dict[str, Any]]:
    records = load_records(path)
    if not records:
        return [], {
            "run_id": _run_id(path),
            "source_path": str(path),
            "records": 0,
            "useful": False,
        }

    flags = _record_flags(
        records,
        dt=dt,
        front_corner_avoid_distance=front_corner_avoid_distance,
        side_avoid_distance=side_avoid_distance,
        max_yaw=max_yaw,
        spin_yaw_threshold=spin_yaw_threshold,
        spin_linear_threshold=spin_linear_threshold,
    )
    intervals = _group_intervals(path, records, flags, dt=dt)
    counts = Counter(interval["failure_type"] for interval in intervals)
    tick_counts = Counter()
    for flag_set in flags:
        for failure_type, active in flag_set.items():
            if active:
                tick_counts[f"{failure_type}_ticks"] += 1

    linear_values = [command(record, "linear_x") for record in records]
    angular_values = [command(record, "angular_z") for record in records]
    metric: Dict[str, Any] = {
        "run_id": _run_id(path),
        "source_path": str(path),
        "records": len(records),
        "duration_s": round(len(records) * dt, 3),
        "useful": True,
        "mean_linear_x": round(_mean(linear_values), 4),
        "mean_abs_angular_z": round(_mean(abs(value) for value in angular_values), 4),
        "max_abs_angular_z": round(max((abs(value) for value in angular_values), default=0.0), 4),
        "interval_count": len(intervals),
    }
    for failure_type in (
        "corner_risk",
        "side_scrape_risk",
        "spin",
        "oscillation",
        "yaw_saturation",
        "recovery_loop",
        "emergency_burst",
        "state_flapping",
    ):
        metric[f"{failure_type}_intervals"] = counts[failure_type]
        metric[f"{failure_type}_ticks"] = tick_counts[f"{failure_type}_ticks"]
    return intervals, metric


def suggested_scenarios(intervals: Iterable[Dict[str, Any]]) -> List[str]:
    kinds = {str(interval.get("failure_type")) for interval in intervals}
    suggestions = []
    if "corner_risk" in kinds:
        suggestions.append("front_left_corner_blocked / front_right_corner_blocked")
        suggestions.append("corner_left_approach / corner_right_approach")
    if "spin" in kinds:
        suggestions.append("spin_trap_open_space")
    if "recovery_loop" in kinds:
        suggestions.append("u_shape_dead_end")
    if "side_scrape_risk" in kinds:
        suggestions.append("wall_too_close_left / wall_too_close_right")
    if "oscillation" in kinds:
        suggestions.append("oscillatory_corridor")
    if "yaw_saturation" in kinds:
        suggestions.append("wall_too_close_left / wall_too_close_right")
    return sorted(set(suggestions))


def resolve_input_paths(paths: List[Path], patterns: Iterable[str]) -> tuple[List[Path], List[str]]:
    searched = [str(path) for path in paths] if paths else list(patterns)
    if paths:
        return sorted({path for path in paths if path.exists()}), searched
    resolved = []
    for pattern in patterns:
        resolved.extend(Path().glob(pattern))
    return sorted({path for path in resolved if path.is_file()}), searched


def _write_jsonl(path: Path, rows: Iterable[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")


def _write_csv(path: Path, rows: List[Dict[str, Any]], default_fieldnames: List[str]) -> None:
    fieldnames = sorted({key for row in rows for key in row}) or default_fieldnames
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_outputs(
    *,
    intervals: List[Dict[str, Any]],
    metrics: List[Dict[str, Any]],
    out_dir: Path,
    searched: List[str],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    intervals_jsonl = out_dir / "failure_intervals.jsonl"
    intervals_csv = out_dir / "failure_intervals.csv"
    metrics_csv = out_dir / "metrics.csv"
    summary_md = out_dir / "summary.md"
    legacy_summary_md = out_dir / "failure_summary.md"
    scenarios_md = out_dir / "suggested_scenarios.md"

    _write_jsonl(intervals_jsonl, intervals)
    _write_csv(intervals_csv, intervals, ["run_id", "failure_type", "notes"])
    _write_csv(metrics_csv, metrics, ["run_id", "records", "useful"])

    counts = Counter(str(interval.get("failure_type")) for interval in intervals)
    tick_totals = Counter()
    for row in metrics:
        for key, value in row.items():
            if key.endswith("_ticks") and isinstance(value, int):
                tick_totals[key] += value

    lines = [
        "# Real Log Failure Analysis",
        "",
        "Offline log analysis only. This does not validate a fix on the robot.",
        "",
        "## Inputs",
        "",
    ]
    for item in searched:
        lines.append(f"- `{item}`")
    lines.extend(["", "## Interval Counts", ""])
    if counts:
        for kind, count in sorted(counts.items()):
            lines.append(f"- `{kind}`: {count}")
    else:
        lines.append("- No failure intervals detected or no useful records found.")
    lines.extend(["", "## Tick Counts", ""])
    for key, count in sorted(tick_totals.items()):
        lines.append(f"- `{key}`: {count}")
    if not tick_totals:
        lines.append("- No failure ticks detected.")

    lines.extend(["", "## Representative Intervals", ""])
    for interval in sorted(intervals, key=lambda item: (str(item["failure_type"]), -float(item["duration_s"])))[:20]:
        lines.append(
            "- `{failure_type}` `{run_id}` idx {start_index}-{end_index}, "
            "{duration_s}s, note: {notes}".format(**interval)
        )

    summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    legacy_summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    scenario_lines = ["# Suggested Synthetic Scenarios", ""]
    suggestions = suggested_scenarios(intervals)
    if suggestions:
        scenario_lines.extend(f"- {item}" for item in suggestions)
    else:
        scenario_lines.append("- No scenario recommendations from supplied logs.")
    scenarios_md.write_text("\n".join(scenario_lines) + "\n", encoding="utf-8")

    print(f"wrote {intervals_jsonl}")
    print(f"wrote {intervals_csv}")
    print(f"wrote {metrics_csv}")
    print(f"wrote {summary_md}")
    print(f"wrote {scenarios_md}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("output/real_log_analysis"))
    parser.add_argument("--pattern", action="append", dest="patterns", help="Glob to search when no paths are provided")
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--front-corner-avoid-distance", type=float, default=0.62)
    parser.add_argument("--side-avoid-distance", type=float, default=0.24)
    parser.add_argument("--max-yaw", type=float, default=0.65)
    parser.add_argument("--spin-yaw-threshold", type=float, default=0.38)
    parser.add_argument("--spin-linear-threshold", type=float, default=0.03)
    args = parser.parse_args()

    patterns = tuple(args.patterns or DEFAULT_PATTERNS)
    paths, searched = resolve_input_paths(args.paths, patterns)
    intervals: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    for path in paths:
        file_intervals, metric = analyze_file(
            path,
            dt=args.dt,
            front_corner_avoid_distance=args.front_corner_avoid_distance,
            side_avoid_distance=args.side_avoid_distance,
            max_yaw=args.max_yaw,
            spin_yaw_threshold=args.spin_yaw_threshold,
            spin_linear_threshold=args.spin_linear_threshold,
        )
        intervals.extend(file_intervals)
        metrics.append(metric)

    if not paths:
        metrics.append(
            {
                "run_id": "NO_LOGS_FOUND",
                "source_path": "",
                "records": 0,
                "useful": False,
                "searched": "; ".join(searched),
            }
        )
        print(f"no real/debug logs found; searched: {', '.join(searched)}")

    write_outputs(intervals=intervals, metrics=metrics, out_dir=args.out_dir, searched=searched)
    useful_files = sum(1 for metric in metrics if metric.get("useful"))
    print(f"analyzed_files={len(paths)} useful_files={useful_files} intervals={len(intervals)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
