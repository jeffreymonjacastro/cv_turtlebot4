#!/usr/bin/env python3
"""Summarize reactive navigator JSONL logs."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from statistics import mean


def load_records(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                print(f"Skipping invalid JSON line {line_number}: {exc}")


def numeric(record, *keys):
    value = record
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="Path to reactive_nav_debug.jsonl")
    args = parser.parse_args()

    records = list(load_records(args.path))
    if not records:
        print("No records found.")
        return 1

    states = Counter(record.get("state", "UNKNOWN") for record in records)
    reasons = Counter(record.get("reason", "UNKNOWN") for record in records)
    requested_yaws = [
        numeric(record, "command", "requested_angular_z")
        for record in records
    ]
    requested_yaws = [value for value in requested_yaws if value is not None]
    published_yaws = [
        numeric(record, "command", "published_angular_z")
        for record in records
    ]
    published_yaws = [value for value in published_yaws if value is not None]
    left_minus_right = [
        numeric(record, "lidar", "left_minus_right_m")
        for record in records
    ]
    left_minus_right = [value for value in left_minus_right if value is not None]

    left_turns = sum(1 for yaw in requested_yaws if yaw > 0.02)
    right_turns = sum(1 for yaw in requested_yaws if yaw < -0.02)
    straight = len(requested_yaws) - left_turns - right_turns

    print(f"Records: {len(records)}")
    print(f"States: {states.most_common()}")
    print(f"Top reasons: {reasons.most_common(5)}")
    if requested_yaws:
        print(
            "Requested yaw: "
            f"mean={mean(requested_yaws):+.3f} "
            f"left_cycles={left_turns} right_cycles={right_turns} straight_cycles={straight}"
        )
    if published_yaws:
        print(f"Published yaw mean: {mean(published_yaws):+.3f}")
    if left_minus_right:
        print(
            "LiDAR left_minus_right_m: "
            f"mean={mean(left_minus_right):+.3f} "
            "(negative means left side is closer than right)"
        )

    print("\nLast 5 records:")
    for record in records[-5:]:
        command = record.get("command", {})
        lidar = record.get("lidar", {})
        nav = record.get("nav", {})
        print(
            f"{record.get('timestamp')} "
            f"state={record.get('state')} reason={record.get('reason')} "
            f"yaw={command.get('requested_angular_z')} "
            f"published_yaw={command.get('published_angular_z')} "
            f"left_minus_right={lidar.get('left_minus_right_m')} "
            f"nav_debug={nav.get('debug')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

