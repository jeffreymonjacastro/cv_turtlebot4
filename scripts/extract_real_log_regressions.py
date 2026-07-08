#!/usr/bin/env python3
"""Extract representative real-log failure intervals as regression cases.

The output cases are small JSONL snippets that keep the original sector fields
and can be replayed by scripts/replay_real_log_nav.py. They are offline
regression artifacts, not claims that a physical issue is fixed.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analyze_robot_failure_log import (  # noqa: E402
    DEFAULT_PATTERNS,
    analyze_file,
    load_records,
    resolve_input_paths,
)


REPRESENTATIVE_TYPES = (
    "corner_risk",
    "spin",
    "oscillation",
    "side_scrape_risk",
    "recovery_loop",
)


def _safe_filename(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)


def _intervals_from_jsonl(path: Path) -> List[Dict[str, Any]]:
    intervals: List[Dict[str, Any]] = []
    if not path.exists():
        return intervals
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict):
                intervals.append(payload)
    return intervals


def _finite_min(*values: Any) -> float:
    finite = [float(value) for value in values if isinstance(value, (int, float)) and math.isfinite(float(value))]
    return min(finite) if finite else math.inf


def _select_representatives(intervals: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    selected: List[Dict[str, Any]] = []
    for failure_type in REPRESENTATIVE_TYPES:
        candidates = [item for item in intervals if item.get("failure_type") == failure_type]
        if not candidates:
            continue
        if failure_type == "corner_risk":
            chosen = min(
                candidates,
                key=lambda item: _finite_min(item.get("min_front_left_m"), item.get("min_front_right_m")),
            )
        elif failure_type == "side_scrape_risk":
            chosen = min(
                candidates,
                key=lambda item: _finite_min(item.get("min_left_m"), item.get("min_right_m")),
            )
        elif failure_type == "oscillation":
            chosen = max(
                candidates,
                key=lambda item: (int(item.get("record_count") or 0), float(item.get("max_abs_angular_z") or 0.0)),
            )
        else:
            chosen = max(candidates, key=lambda item: float(item.get("duration_s") or 0.0))
        selected.append(chosen)
    return selected


def _case_name(interval: Dict[str, Any], ordinal: int) -> str:
    raw_type = str(interval.get("failure_type") or "failure")
    short_type = {
        "side_scrape_risk": "side_scrape",
        "corner_risk": "corner_risk",
        "recovery_loop": "recovery_loop",
    }.get(raw_type, raw_type)
    run_id = _safe_filename(str(interval.get("run_id") or Path(str(interval.get("source_path") or "run")).stem))
    return f"real_{short_type}_{run_id}_{ordinal:03d}"


def _copy_records_for_interval(
    interval: Dict[str, Any],
    *,
    dt: float,
    padding_s: float,
    max_records: int,
) -> List[dict]:
    path = Path(str(interval.get("source_path") or ""))
    records = load_records(path)
    if not records:
        return []
    padding = max(0, int(round(padding_s / dt)))
    start = max(0, int(interval.get("start_index") or 0) - padding)
    end = min(len(records) - 1, int(interval.get("end_index") or start) + padding)
    chosen = records[start : end + 1]
    if len(chosen) > max_records:
        center = int(interval.get("representative_record_index") or start)
        half = max_records // 2
        start = max(0, center - half)
        end = min(len(records), start + max_records)
        start = max(0, end - max_records)
        chosen = records[start:end]
    return [dict(record) for record in chosen]


def _write_case(path: Path, case: Dict[str, Any], records: List[dict], dt: float) -> None:
    with path.open("w", encoding="utf-8") as handle:
        metadata = {
            "record_type": "metadata",
            "scenario": case["name"],
            "source_log": case["source_path"],
            "source_failure_type": case["failure_type"],
            "dt_s": dt,
            "duration_s": round(len(records) * dt, 3),
            "replay_type": "real_failure_interval_regression",
            "dry_run": True,
            "enable_motion": False,
        }
        handle.write(json.dumps(metadata, ensure_ascii=True, sort_keys=True) + "\n")
        for index, record in enumerate(records):
            payload = dict(record)
            payload["record_type"] = "step"
            payload["scenario"] = case["name"]
            payload["time_s"] = round(index * dt, 3)
            payload["dt_s"] = dt
            handle.write(json.dumps(payload, ensure_ascii=True, sort_keys=True) + "\n")


def _build_intervals_from_logs(paths: List[Path], dt: float) -> List[Dict[str, Any]]:
    intervals: List[Dict[str, Any]] = []
    for path in paths:
        file_intervals, _metric = analyze_file(
            path,
            dt=dt,
            front_corner_avoid_distance=0.62,
            side_avoid_distance=0.24,
            max_yaw=0.65,
            spin_yaw_threshold=0.38,
            spin_linear_threshold=0.03,
        )
        intervals.extend(file_intervals)
    return intervals


def _write_summary(path: Path, cases: List[Dict[str, Any]], searched: Iterable[str]) -> None:
    lines = [
        "# Real-Log Regression Cases",
        "",
        "Offline regression snippets extracted from real/debug logs.",
        "",
        "## Inputs",
        "",
    ]
    for item in searched:
        lines.append(f"- `{item}`")
    lines.extend(["", "## Cases", ""])
    if cases:
        lines.extend(
            "- `{name}` from `{failure_type}` in `{source_path}` records {start_index}-{end_index}".format(**case)
            for case in cases
        )
    else:
        lines.append("- No representative intervals were available.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path)
    parser.add_argument("--intervals", type=Path, default=Path("output/real_log_analysis/failure_intervals.jsonl"))
    parser.add_argument("--out-dir", type=Path, default=Path("output/real_log_regressions"))
    parser.add_argument("--pattern", action="append", dest="patterns")
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--padding-s", type=float, default=0.5)
    parser.add_argument("--max-records", type=int, default=80)
    args = parser.parse_args()

    patterns = tuple(args.patterns or DEFAULT_PATTERNS)
    paths, searched = resolve_input_paths(args.paths, patterns)
    intervals = _intervals_from_jsonl(args.intervals)
    if not intervals:
        intervals = _build_intervals_from_logs(paths, args.dt)

    representatives = _select_representatives(intervals)
    cases_dir = args.out_dir / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    cases: List[Dict[str, Any]] = []
    for ordinal, interval in enumerate(representatives, start=1):
        records = _copy_records_for_interval(
            interval,
            dt=args.dt,
            padding_s=args.padding_s,
            max_records=args.max_records,
        )
        if not records:
            continue
        case = {
            "name": _case_name(interval, ordinal),
            "failure_type": interval.get("failure_type"),
            "source_path": interval.get("source_path"),
            "start_index": interval.get("start_index"),
            "end_index": interval.get("end_index"),
            "duration_s": interval.get("duration_s"),
            "record_count": len(records),
            "case_path": str(cases_dir / f"{_case_name(interval, ordinal)}.jsonl"),
            "notes": interval.get("notes"),
        }
        _write_case(Path(case["case_path"]), case, records, args.dt)
        cases.append(case)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    scenarios_json = args.out_dir / "regression_scenarios.json"
    summary_md = args.out_dir / "summary.md"
    scenarios_json.write_text(json.dumps(cases, indent=2, sort_keys=True), encoding="utf-8")
    _write_summary(summary_md, cases, searched)
    for case in cases:
        print(f"wrote {case['case_path']}")
    print(f"wrote {scenarios_json}")
    print(f"wrote {summary_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
