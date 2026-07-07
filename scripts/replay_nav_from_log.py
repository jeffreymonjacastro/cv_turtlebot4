#!/usr/bin/env python3
"""Replay navigation modules from sector-level robot/debug JSONL logs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.compare_nav_profiles import summarize  # noqa: E402
from scripts.replay_nav_scenarios import (  # noqa: E402
    build_arbiter,
    corridor_scan,
    load_replay_profile,
    nav_kwargs,
    _json_clean,
    _sector_record,
)
from ubuntu.reactive_nav.behavior_arbiter import ArbiterInput, SignalState  # noqa: E402
from ubuntu.reactive_nav.lidar_sectors import extract_sectors  # noqa: E402
from ubuntu.reactive_nav.wall_following import NavigationObservation, create_navigation_module  # noqa: E402


def load_records(path: Path) -> List[dict]:
    records = []
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


def sector(record: dict, name: str, default: float) -> float:
    for path in (
        ("lidar", f"{name}_m"),
        ("lidar", "sector_distance_m", name),
        ("lidar", name),
    ):
        value = number(record, *path)
        if value is not None:
            return value
    return default


def scan_from_record(record: dict, t: float):
    return corridor_scan(
        front=sector(record, "front", 1.5),
        front_center=sector(record, "front_center", sector(record, "front", 1.5)),
        front_left=sector(record, "front_left", 1.2),
        front_right=sector(record, "front_right", 1.2),
        left=sector(record, "left", 0.7),
        right=sector(record, "right", 0.7),
        rear=sector(record, "rear", 1.0),
        stamp=t,
    )


def signal_from_record(record: dict, now: float) -> SignalState:
    payload = record.get("signal") if isinstance(record.get("signal"), dict) else {}
    direction = str(payload.get("direction") or "none")
    stale = bool(payload.get("stale", True))
    actionable = bool(payload.get("actionable", False))
    return SignalState(
        direction=direction,
        confidence=float(payload.get("confidence") or 0.0),
        bbox_area_ratio=float(payload.get("bbox_area_ratio") or 0.0),
        bbox_center_x_ratio=float(payload.get("bbox_center_x_ratio") or 0.5),
        actionable=actionable,
        timestamp=now,
        stale=stale,
        event_id=str(payload.get("event_id") or ""),
        reason=str(payload.get("reason") or "log_replay"),
    )


def replay_file(path: Path, module_name: str, out_dir: Path, dt_s: float) -> Path:
    records = load_records(path)
    params = load_replay_profile(module_name)
    params["profile_name"] = f"{module_name}_log_replay"
    nav = create_navigation_module(module_name, **nav_kwargs(params))
    arbiter = build_arbiter(params)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{path.stem}__{module_name}_sector_replay.jsonl"
    sim_start = time.monotonic()
    previous_state = ""

    with out_path.open("w", encoding="utf-8") as handle:
        metadata = {
            "record_type": "metadata",
            "scenario": f"log_replay:{path.name}",
            "profile_name": params["profile_name"],
            "nav_module": module_name,
            "dt_s": dt_s,
            "duration_s": len(records) * dt_s,
            "source_log": str(path),
            "replay_type": "sector_level_log_replay",
            "dry_run": True,
            "enable_motion": False,
            "config": {k: v for k, v in params.items() if not k.startswith("_")},
        }
        handle.write(json.dumps(_json_clean(metadata), sort_keys=True) + "\n")
        for index, record in enumerate(records):
            t = index * dt_s
            now = sim_start + t
            scan = scan_from_record(record, t)
            sectors = extract_sectors(scan)
            lidar_fresh = bool(nested(record, "freshness", "lidar_fresh") if nested(record, "freshness", "lidar_fresh") is not None else True)
            nav_suggestion = nav.compute(NavigationObservation(sectors, now, dt_s)) if lidar_fresh else None
            output = arbiter.decide(
                ArbiterInput(
                    sectors=sectors,
                    lidar_fresh=lidar_fresh,
                    nav_suggestion=nav_suggestion,
                    signal=signal_from_record(record, now),
                    qr_recent=bool(nested(record, "qr", "visible")),
                    now=now,
                )
            )
            replay_record = {
                "record_type": "step",
                "timestamp": round(t, 3),
                "time_s": round(t, 3),
                "dt_s": dt_s,
                "scenario": f"log_replay:{path.name}",
                "profile_name": params["profile_name"],
                "state": output.state,
                "previous_state": previous_state,
                "reason": output.reason,
                "source_log_state": record.get("state"),
                "source_log_reason": record.get("reason"),
                "nav": {
                    "module": module_name,
                    "suggestion_mode": nav_suggestion.mode if nav_suggestion else None,
                    "suggestion_reason": nav_suggestion.reason if nav_suggestion else None,
                    "suggested_linear_x": nav_suggestion.command.linear_x if nav_suggestion else None,
                    "suggested_angular_z": nav_suggestion.command.angular_z if nav_suggestion else None,
                    "debug": nav_suggestion.debug if nav_suggestion else {},
                },
                "arbiter_debug": output.debug,
                "lidar": _sector_record(sectors, 0.0 if lidar_fresh else 999.0, lidar_fresh),
                "freshness": {"lidar_fresh": lidar_fresh, "lidar_age_s": 0.0 if lidar_fresh else 999.0},
                "signal": record.get("signal", {}),
                "qr": record.get("qr", {}),
                "command": {
                    "requested_linear_x": output.command.linear_x,
                    "requested_angular_z": output.command.angular_z,
                    "published_linear_x": output.command.linear_x,
                    "published_angular_z": output.command.angular_z,
                    "motion_published_to_robot": False,
                    "publication_mode": "sector_level_log_replay",
                },
                "dry_run": True,
                "enable_motion": False,
            }
            handle.write(json.dumps(_json_clean(replay_record), sort_keys=True) + "\n")
            previous_state = output.state
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--nav-modules", nargs="+", default=["wall_follow", "follow_gap", "focm"])
    parser.add_argument("--out-dir", type=Path, default=Path("output/log_replay_runs"))
    parser.add_argument("--dt", type=float, default=0.1)
    args = parser.parse_args()

    written = []
    for path in args.paths:
        if not path.exists():
            continue
        for module_name in args.nav_modules:
            written.append(replay_file(path, module_name, args.out_dir, args.dt))
    summaries = [summarize(path) for path in written]
    summary_path = args.out_dir / "sector_replay_summary.json"
    summary_path.write_text(json.dumps(summaries, indent=2, sort_keys=True), encoding="utf-8")
    for path in written:
        print(f"wrote {path}")
    print(f"wrote {summary_path}")
    print("sector-level/log replay only; not physical validation")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
