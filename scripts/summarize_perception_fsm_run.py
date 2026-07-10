#!/usr/bin/env python3
"""Summarize a perception/FSM supervision run directory or JSONL log."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any, Dict, Iterable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="Run directory or reactive_nav_debug.jsonl path.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON summary.")
    return parser.parse_args()


def log_paths(path: Path) -> Iterable[Path]:
    if path.is_file():
        yield path
        return
    yield from sorted(path.glob("**/reactive_nav_debug.jsonl"))


def read_records(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def summarize(path: Path) -> Dict[str, Any]:
    records = []
    for log_path in log_paths(path):
        records.extend(read_records(log_path))

    states = Counter()
    reasons = Counter()
    yolo_events = Counter()
    yolo_status = Counter()
    qr_events = Counter()
    qr_status = Counter()
    command_sources = Counter()
    unsafe_published = 0
    blocked_cycles = 0

    for record in records:
        sup = record.get("supervision") if isinstance(record.get("supervision"), dict) else {}
        states[str(sup.get("current_state") or record.get("state") or "UNKNOWN")] += 1
        reasons[str(sup.get("transition_reason") or record.get("reason") or "UNKNOWN")] += 1
        yolo_events[str(sup.get("yolo_event") or "NONE")] += 1
        yolo_status[str(sup.get("yolo_event_status") or "unknown")] += 1
        qr_events[str(sup.get("qr_event") or "NONE")] += 1
        qr_status[str(sup.get("qr_event_status") or "unknown")] += 1
        command_sources[str(sup.get("command_source") or "unknown")] += 1
        published_linear = float(sup.get("published_linear_x") or 0.0)
        published_yaw = float(sup.get("published_angular_z") or 0.0)
        if bool(sup.get("dry_run")) or not bool(sup.get("enable_motion")):
            if abs(published_linear) > 1e-6 or abs(published_yaw) > 1e-6:
                unsafe_published += 1
        if (
            abs(float(sup.get("arbiter_linear_x") or 0.0)) > 1e-6
            or abs(float(sup.get("arbiter_angular_z") or 0.0)) > 1e-6
        ) and abs(published_linear) <= 1e-6 and abs(published_yaw) <= 1e-6:
            blocked_cycles += 1

    return {
        "path": str(path),
        "record_count": len(records),
        "states": dict(states.most_common()),
        "transition_reasons": dict(reasons.most_common(10)),
        "yolo_events": dict(yolo_events.most_common()),
        "yolo_event_status": dict(yolo_status.most_common()),
        "qr_events": dict(qr_events.most_common()),
        "qr_event_status": dict(qr_status.most_common()),
        "command_sources": dict(command_sources.most_common()),
        "intended_command_blocked_cycles": blocked_cycles,
        "unsafe_nonzero_published_while_blocked": unsafe_published,
        "motion_safely_blocked": unsafe_published == 0,
    }


def print_text(summary: Dict[str, Any]) -> None:
    print(f"Run: {summary['path']}")
    print(f"Records: {summary['record_count']}")
    print(f"States: {summary['states']}")
    print(f"Transition reasons: {summary['transition_reasons']}")
    print(f"YOLO events: {summary['yolo_events']}")
    print(f"YOLO status: {summary['yolo_event_status']}")
    print(f"QR events: {summary['qr_events']}")
    print(f"QR status: {summary['qr_event_status']}")
    print(f"Command sources: {summary['command_sources']}")
    print(f"Intended command blocked cycles: {summary['intended_command_blocked_cycles']}")
    print(f"Unsafe non-zero published while blocked: {summary['unsafe_nonzero_published_while_blocked']}")
    print(f"Motion safely blocked: {summary['motion_safely_blocked']}")


def main() -> int:
    args = parse_args()
    summary = summarize(Path(args.path))
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print_text(summary)
    return 0 if summary["record_count"] > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
