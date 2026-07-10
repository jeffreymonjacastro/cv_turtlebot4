#!/usr/bin/env python3
"""Lightweight terminal supervision view for reactive-nav JSONL logs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any, Dict


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log_path",
        nargs="?",
        default="output/reactive_nav_debug.jsonl",
        help="Path to reactive_nav_debug.jsonl.",
    )
    parser.add_argument("--follow", action="store_true", help="Continue waiting for new records.")
    parser.add_argument("--interval", type=float, default=0.2, help="Follow polling interval.")
    return parser.parse_args()


def _get(record: Dict[str, Any], *path: str, default=None):
    value: Any = record
    for key in path:
        if not isinstance(value, dict):
            return default
        value = value.get(key)
    return default if value is None else value


def render(record: Dict[str, Any]) -> str:
    sup = record.get("supervision") if isinstance(record.get("supervision"), dict) else {}
    lines = [
        f"time={record.get('timestamp')} state={sup.get('current_state') or record.get('state')} "
        f"prev={sup.get('previous_state') or record.get('previous_state')} reason={sup.get('transition_reason') or record.get('reason')}",
        f"YOLO raw={sup.get('raw_yolo_class')} conf={sup.get('raw_yolo_confidence')} "
        f"age={sup.get('raw_yolo_age_s')} progress={sup.get('yolo_confirmation_progress')} "
        f"event={sup.get('yolo_event')} status={sup.get('yolo_event_status')} "
        f"reject={sup.get('yolo_rejection_reason')}",
        f"QR raw={sup.get('raw_qr_payload')} decode={sup.get('qr_decode_status')} "
        f"progress={sup.get('qr_confirmation_progress')} event={sup.get('qr_event')} "
        f"status={sup.get('qr_event_status')} reject={sup.get('qr_rejection_reason')}",
        f"maneuver active={sup.get('active_maneuver')} phase={sup.get('maneuver_phase')} "
        f"elapsed={sup.get('maneuver_elapsed_s')}",
        f"cmd source={sup.get('command_source')} suggested=({sup.get('suggested_linear_x')}, {sup.get('suggested_angular_z')}) "
        f"arbiter=({sup.get('arbiter_linear_x')}, {sup.get('arbiter_angular_z')}) "
        f"published=({sup.get('published_linear_x')}, {sup.get('published_angular_z')}) "
        f"dry_run={sup.get('dry_run')} enable_motion={sup.get('enable_motion')}",
    ]
    return "\n".join(lines)


def iter_records(path: Path, *, follow: bool, interval: float):
    position = 0
    while True:
        if path.exists():
            with path.open("r", encoding="utf-8") as handle:
                handle.seek(position)
                for line in handle:
                    if not line.strip():
                        continue
                    try:
                        yield json.loads(line)
                    except json.JSONDecodeError:
                        continue
                position = handle.tell()
        if not follow:
            break
        time.sleep(max(0.05, interval))


def main() -> int:
    args = parse_args()
    path = Path(args.log_path)
    seen = False
    for record in iter_records(path, follow=args.follow, interval=args.interval):
        seen = True
        print(render(record))
        print("-" * 80)
        sys.stdout.flush()
    if not seen and not args.follow:
        print(f"no records found in {path}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
