#!/usr/bin/env python3
"""Summarize reactive navigation JSONL runs for profile comparison."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path
import json
from statistics import mean
from typing import Dict, Iterable, List


def load_records(path: Path) -> List[dict]:
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"{path}: skipping invalid JSON line {line_number}: {exc}")
    return records


def numeric(record: dict, *keys: str) -> float | None:
    value = record
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def is_turn_state(state: str) -> bool:
    return state.startswith("TURNING_") or state in {"SETTLING_AFTER_TURN", "ALIGNING_AFTER_TURN"}


def summarize(path: Path) -> Dict[str, object]:
    records = load_records(path)
    if not records:
        return {"path": str(path), "error": "no_records"}

    total_runtime = sum(numeric(record, "dt_s") or 0.0 for record in records)
    state_time = defaultdict(float)
    state_counts = Counter()
    corridor_speeds: List[float] = []
    emergency_stop_time = 0.0
    emergency_stop_count = 0
    turn_durations: List[float] = []
    turn_active = False
    turn_elapsed = 0.0

    for index, record in enumerate(records):
        state = str(record.get("state", "UNKNOWN"))
        dt = numeric(record, "dt_s") or 0.0
        state_counts[state] += 1
        state_time[state] += dt

        previous_state = str(record.get("previous_state") or "")
        if state == "EMERGENCY_STOP":
            emergency_stop_time += dt
            if previous_state != "EMERGENCY_STOP":
                emergency_stop_count += 1

        published_linear = numeric(record, "command", "published_linear_x")
        if state == "CORRIDOR_FOLLOW" and published_linear is not None:
            corridor_speeds.append(published_linear)

        if is_turn_state(state):
            turn_elapsed += dt
            turn_active = True
        elif turn_active:
            turn_durations.append(turn_elapsed)
            turn_elapsed = 0.0
            turn_active = False

        if index == len(records) - 1 and turn_active:
            turn_durations.append(turn_elapsed)

    return {
        "path": str(path),
        "profile_name": records[-1].get("profile_name"),
        "nav_module": records[-1].get("nav", {}).get("module"),
        "records": len(records),
        "total_runtime_s": total_runtime,
        "state_counts": dict(state_counts),
        "state_time_s": dict(sorted(state_time.items())),
        "emergency_stop_count": emergency_stop_count,
        "emergency_stop_time_s": emergency_stop_time,
        "corridor_follow_avg_linear_x": mean(corridor_speeds) if corridor_speeds else None,
        "recovery_time_ratio": (state_time["RECOVERY"] / total_runtime) if total_runtime > 0.0 else 0.0,
        "turn_count": len(turn_durations),
        "turn_avg_completion_s": mean(turn_durations) if turn_durations else None,
    }


def format_summary(summary: Dict[str, object]) -> Iterable[str]:
    if "error" in summary:
        yield f"{summary['path']}: {summary['error']}"
        return

    corridor_avg = summary["corridor_follow_avg_linear_x"]
    turn_avg = summary["turn_avg_completion_s"]
    yield (
        f"{summary['path']}\n"
        f"  profile={summary['profile_name']} nav_module={summary['nav_module']} records={summary['records']}\n"
        f"  runtime={summary['total_runtime_s']:.2f}s emergency_count={summary['emergency_stop_count']} "
        f"emergency_time={summary['emergency_stop_time_s']:.2f}s\n"
        f"  corridor_avg_linear_x={'n/a' if corridor_avg is None else f'{corridor_avg:.3f}'} "
        f"recovery_ratio={summary['recovery_time_ratio']:.3f}\n"
        f"  turns={summary['turn_count']} "
        f"turn_avg={'n/a' if turn_avg is None else f'{turn_avg:.2f}s'}\n"
        f"  state_time_s={summary['state_time_s']}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path, help="One or more reactive_nav_debug.jsonl files")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    args = parser.parse_args()

    summaries = [summarize(path) for path in args.paths]
    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))
        return 0

    for summary in summaries:
        for block in format_summary(summary):
            print(block)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
