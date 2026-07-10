#!/usr/bin/env python3
"""Extract turn/recovery failure intervals from reactive-navigation JSONL logs.

This is an offline analyzer.  It reads debug logs only and never imports ROS or
publishes a command.  The output is suitable for sector-level replay, while
remaining explicit when a source log contains no real turn state.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
import glob
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any, Iterable, List, Optional


TURN_STATES = {"TURNING_LEFT", "TURNING_RIGHT", "TURNING_UTURN", "SETTLING_AFTER_TURN", "ALIGNING_AFTER_TURN"}
RECOVERY_STATE = "RECOVERY"


def load_records(path: Path) -> List[dict]:
    records: List[dict] = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(record, dict) and record.get("record_type") != "metadata":
                records.append(record)
    except OSError:
        pass
    return records


def _nested(record: dict, *keys: str) -> Any:
    value: Any = record
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _number(record: dict, *paths: tuple[str, ...]) -> Optional[float]:
    for path in paths:
        value = _nested(record, *path)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return float(value)
    return None


def sector(record: dict, name: str) -> Optional[float]:
    return _number(
        record,
        ("lidar", "sector_distance_m", name),
        ("lidar", f"{name}_m"),
        ("lidar", name),
    )


def command(record: dict, name: str) -> float:
    return _number(
        record,
        ("command", f"published_{name}"),
        ("command", f"requested_{name}"),
        ("nav", f"suggested_{name}"),
    ) or 0.0


def record_time(record: dict, index: int, dt_s: float) -> float:
    value = _number(record, ("time_s",), ("timestamp",))
    return value if value is not None else index * dt_s


def _min(values: Iterable[Optional[float]]) -> Optional[float]:
    valid = [value for value in values if value is not None and math.isfinite(value)]
    return min(valid) if valid else None


def _compressed_states(records: List[dict]) -> List[str]:
    sequence: List[str] = []
    for record in records:
        state = str(record.get("state") or "UNKNOWN")
        if not sequence or sequence[-1] != state:
            sequence.append(state)
    return sequence


def _gap_sign(record: dict) -> int:
    value = _number(
        record,
        ("recovery", "best_gap_center_deg"),
        ("nav", "debug", "gap_center"),
    )
    if value is None or abs(value) < 1.0:
        return 0
    return 1 if value > 0.0 else -1


def _suspected_cause(selected: List[dict], *, recovery_entries: int, front_selects: int, gap_flips: int) -> str:
    front = _min(sector(record, "front") for record in selected)
    left_corner = _min(sector(record, "front_left") for record in selected)
    right_corner = _min(sector(record, "front_right") for record in selected)
    if gap_flips >= 2:
        return "selected_gap_direction_flapping"
    if recovery_entries >= 3:
        return "recovery_state_loop"
    if front_selects >= 3:
        return "repeated_front_blocked_gap_selection"
    if front is not None and front < 0.35:
        return "emergency_or_front_clearance_limited"
    if left_corner is not None and right_corner is not None and min(left_corner, right_corner) < 0.45:
        return "corner_clearance_or_angle_offset_candidate"
    return "insufficient_evidence_capture_more_turn_diagnostics"


def summarize_interval(path: Path, records: List[dict], start: int, end: int, dt_s: float, interval_id: int) -> dict:
    selected = records[start : end + 1]
    states = _compressed_states(selected)
    recovery_entries = sum(
        1
        for previous, current in zip(["INIT"] + [str(record.get("state") or "UNKNOWN") for record in selected], [str(record.get("state") or "UNKNOWN") for record in selected])
        if current == RECOVERY_STATE and previous != RECOVERY_STATE
    )
    front_selects = sum(
        1 for record in selected if str(record.get("reason") or "") == "FRONT_BLOCKED_SELECT_FREE_GAP"
    )
    signs = [sign for sign in (_gap_sign(record) for record in selected) if sign]
    gap_flips = sum(1 for previous, current in zip(signs, signs[1:]) if previous != current)
    start_time = record_time(records[start], start, dt_s)
    end_time = record_time(records[end], end, dt_s) + dt_s
    yaw = [command(record, "angular_z") for record in selected]
    linear = [command(record, "linear_x") for record in selected]
    turn_count = sum(1 for record in selected if str(record.get("state") or "") in TURN_STATES)
    return {
        "interval_id": f"turn_recovery_{interval_id:03d}",
        "source_log": str(path),
        "run_name": path.stem,
        "start_index": start,
        "end_index": end,
        "start_time": round(start_time, 3),
        "end_time": round(end_time, 3),
        "duration_s": round(max(dt_s, end_time - start_time), 3),
        "initial_state": str(selected[0].get("state") or "UNKNOWN"),
        "terminal_state": str(selected[-1].get("state") or "UNKNOWN"),
        "state_sequence": states,
        "profile_name": str(selected[0].get("profile_name") or "unknown"),
        "nav_module": str(_nested(selected[0], "nav", "module") or "unknown"),
        "min_front": _min(sector(record, "front") for record in selected),
        "min_front_left": _min(sector(record, "front_left") for record in selected),
        "min_front_right": _min(sector(record, "front_right") for record in selected),
        "min_left": _min(sector(record, "left") for record in selected),
        "min_right": _min(sector(record, "right") for record in selected),
        "max_abs_yaw": round(max((abs(value) for value in yaw), default=0.0), 4),
        "avg_linear": round(mean(linear) if linear else 0.0, 4),
        "turn_tick_count": turn_count,
        "recovery_entry_count": recovery_entries,
        "front_blocked_select_count": front_selects,
        "gap_direction_flip_count": gap_flips,
        "contains_turn": any(item in TURN_STATES for item in states),
        "suspected_cause": _suspected_cause(
            selected, recovery_entries=recovery_entries, front_selects=front_selects, gap_flips=gap_flips
        ),
    }


def extract_intervals(records: List[dict], path: Path, *, dt_s: float, min_recovery_s: float) -> List[dict]:
    intervals: List[dict] = []
    start: Optional[int] = None
    quiet_ticks = 0
    for index, record in enumerate(records):
        current_state = str(record.get("state") or "UNKNOWN")
        current_reason = str(record.get("reason") or "")
        active = current_state in TURN_STATES or current_state == RECOVERY_STATE or current_reason == "FRONT_BLOCKED_SELECT_FREE_GAP"
        if active:
            if start is None:
                start = index
            quiet_ticks = 0
            continue
        if start is None:
            continue
        quiet_ticks += 1
        if quiet_ticks < 2:
            continue
        end = index - quiet_ticks
        candidate = summarize_interval(path, records, start, end, dt_s, len(intervals) + 1)
        # This focused artifact is for recovery failures. Normal completed turns
        # are deliberately excluded; they are not evidence of a turn/recovery bug.
        if candidate["recovery_entry_count"] and (
            candidate["contains_turn"]
            or candidate["duration_s"] >= min_recovery_s
            or candidate["recovery_entry_count"] >= 2
        ):
            intervals.append(candidate)
        start = None
        quiet_ticks = 0
    if start is not None:
        candidate = summarize_interval(path, records, start, len(records) - 1, dt_s, len(intervals) + 1)
        if candidate["recovery_entry_count"] and (
            candidate["contains_turn"]
            or candidate["duration_s"] >= min_recovery_s
            or candidate["recovery_entry_count"] >= 2
        ):
            intervals.append(candidate)
    return intervals


def _write_csv(path: Path, rows: List[dict]) -> None:
    fields = sorted({key for row in rows for key in row}) or ["interval_id"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            normalized = dict(row)
            normalized["state_sequence"] = " -> ".join(row.get("state_sequence") or [])
            writer.writerow(normalized)


def _write_markdown(path: Path, intervals: List[dict], source_count: int) -> None:
    causes = Counter(str(interval["suspected_cause"]) for interval in intervals)
    lines = [
        "# Turn/Recovery Failure Analysis",
        "",
        "Offline log analysis only. It does not validate robot behavior or publish `/cmd_vel`.",
        "",
        f"- source logs: {source_count}",
        f"- extracted intervals: {len(intervals)}",
        f"- intervals containing recorded turn states: {sum(bool(item['contains_turn']) for item in intervals)}",
        "",
        "| interval | duration | sequence | recovery entries | front selects | suspected cause |",
        "| --- | ---: | --- | ---: | ---: | --- |",
    ]
    for item in intervals:
        lines.append(
            f"| {item['interval_id']} | {item['duration_s']:.2f} | {' → '.join(item['state_sequence'])} | "
            f"{item['recovery_entry_count']} | {item['front_blocked_select_count']} | {item['suspected_cause']} |"
        )
    lines.extend(["", "## Suggested Changes", ""])
    if not intervals:
        lines.append("- No qualifying intervals found. Capture an isolated turn with the new diagnostics enabled.")
    else:
        for cause, count in causes.most_common():
            lines.append(f"- `{cause}`: {count} interval(s); replay before changing controller behavior.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def resolve_paths(items: List[str]) -> List[Path]:
    paths: List[Path] = []
    for item in items:
        matches = [Path(match) for match in glob.glob(item)]
        if matches:
            paths.extend(matches)
        elif Path(item).is_file():
            paths.append(Path(item))
    return sorted(set(paths))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help="JSONL paths or glob patterns")
    parser.add_argument("--out-dir", type=Path, default=Path("output/turn_recovery_analysis"))
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--min-recovery-s", type=float, default=1.0)
    args = parser.parse_args()

    paths = resolve_paths(args.paths)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    all_intervals: List[dict] = []
    for source in paths:
        all_intervals.extend(extract_intervals(load_records(source), source, dt_s=args.dt, min_recovery_s=args.min_recovery_s))
    for index, item in enumerate(all_intervals, start=1):
        item["interval_id"] = f"turn_recovery_{index:03d}"

    jsonl_path = args.out_dir / "failure_intervals.jsonl"
    with jsonl_path.open("w", encoding="utf-8") as handle:
        for item in all_intervals:
            handle.write(json.dumps(item, ensure_ascii=True, sort_keys=True) + "\n")
    _write_csv(args.out_dir / "failure_intervals.csv", all_intervals)
    _write_markdown(args.out_dir / "failure_summary.md", all_intervals, len(paths))
    _write_markdown(args.out_dir / "suggested_changes.md", all_intervals, len(paths))
    print(f"wrote {jsonl_path}")
    print("offline analysis only; no /cmd_vel publication")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
