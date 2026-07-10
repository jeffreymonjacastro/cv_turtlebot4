#!/usr/bin/env python3
"""Replay extracted turn/recovery intervals through a navigation profile.

The replay reconstructs observations from logged LiDAR sectors, so it is
explicitly sector-level.  It is useful for comparing arbitration decisions but
does not validate raw scan geometry, LiDAR angle offset, or physical motion.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import mean
import sys
import time
from typing import Any, Dict, Iterable, List


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analyze_robot_failure_log import _record_flags, command, load_records, nested  # noqa: E402
from scripts.replay_nav_scenarios import _json_clean, _sector_record, build_arbiter, nav_kwargs  # noqa: E402
from scripts.replay_real_log_nav import _profile_params, scan_from_record, signal_from_record  # noqa: E402
from ubuntu.reactive_nav.behavior_arbiter import ArbiterInput  # noqa: E402
from ubuntu.reactive_nav.lidar_sectors import extract_sectors  # noqa: E402
from ubuntu.reactive_nav.wall_following import NavigationObservation, create_navigation_module  # noqa: E402


TURN_STATES = {"TURNING_LEFT", "TURNING_RIGHT", "TURNING_UTURN", "SETTLING_AFTER_TURN", "ALIGNING_AFTER_TURN"}
_SOURCE_RECORD_CACHE: Dict[Path, List[dict]] = {}


def load_intervals(path: Path) -> List[dict]:
    intervals: List[dict] = []
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(item, dict):
                intervals.append(item)
    except OSError:
        pass
    return intervals


def _safe_name(value: str) -> str:
    return "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in value)


def _source_records(path: Path) -> List[dict]:
    """Cache source logs so a multi-ablation run reads each large log once."""
    if path not in _SOURCE_RECORD_CACHE:
        _SOURCE_RECORD_CACHE[path] = load_records(path)
    return _SOURCE_RECORD_CACHE[path]


def _duration_in_state(records: Iterable[dict], state: str, dt_s: float) -> float:
    return sum(dt_s for record in records if record.get("state") == state)


def _recovery_entries(records: List[dict]) -> int:
    states = [str(record.get("state") or "UNKNOWN") for record in records]
    return sum(1 for previous, current in zip(["INIT"] + states, states) if current == "RECOVERY" and previous != "RECOVERY")


def _recovery_entries_after_turn(records: List[dict]) -> int:
    seen_turn = False
    previous_state = "INIT"
    entries = 0
    for record in records:
        current_state = str(record.get("state") or "UNKNOWN")
        seen_turn = seen_turn or current_state in TURN_STATES
        if seen_turn and current_state == "RECOVERY" and previous_state != "RECOVERY":
            entries += 1
        previous_state = current_state
    return entries


def _max_recovery_duration(records: List[dict], dt_s: float) -> float:
    best = current = 0.0
    for record in records:
        if record.get("state") == "RECOVERY":
            current += dt_s
            best = max(best, current)
        else:
            current = 0.0
    return round(best, 3)


def replay_interval(
    interval: dict,
    profile_name: str,
    out_dir: Path,
    *,
    dt_s: float = 0.1,
    params_override: Dict[str, Any] | None = None,
    variant_name: str | None = None,
) -> Dict[str, Any]:
    """Replay one interval and return comparable offline-only metrics."""
    source = Path(str(interval["source_log"]))
    source_records = _source_records(source)
    start = max(0, int(interval["start_index"]))
    end = min(len(source_records) - 1, int(interval["end_index"]))
    target_records = source_records[start : end + 1]
    module_name, params = _profile_params(profile_name)
    params = dict(params)
    if params_override:
        params.update(params_override)
    profile_label = variant_name or profile_name
    nav = create_navigation_module(module_name, **nav_kwargs(params))
    arbiter = build_arbiter(params)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{_safe_name(str(interval['interval_id']))}__{_safe_name(profile_label)}.jsonl"
    sim_start = time.monotonic()
    replay_records: List[dict] = []
    previous_state = ""

    with out_path.open("w", encoding="utf-8") as handle:
        metadata = {
            "record_type": "metadata",
            "interval_id": interval["interval_id"],
            "source_log": str(source),
            "source_range": {"start_index": start, "end_index": end},
            "profile_name": profile_label,
            "nav_module": module_name,
            "replay_type": "sector_level",
            "angle_offset_validated": False,
            "dry_run": True,
            "enable_motion": False,
            "command_publication": "disabled_offline_replay",
            "config": {key: value for key, value in params.items() if not key.startswith("_")},
        }
        handle.write(json.dumps(_json_clean(metadata), ensure_ascii=True, sort_keys=True) + "\n")
        for local_index, record in enumerate(target_records):
            now = sim_start + local_index * dt_s
            scan = scan_from_record(record, local_index * dt_s)
            sectors = extract_sectors(scan)
            lidar_fresh = nested(record, "freshness", "lidar_fresh")
            lidar_fresh = True if lidar_fresh is None else bool(lidar_fresh)
            suggestion = nav.compute(NavigationObservation(sectors, now, dt_s)) if lidar_fresh else None
            output = arbiter.decide(
                ArbiterInput(
                    sectors=sectors,
                    lidar_fresh=lidar_fresh,
                    nav_suggestion=suggestion,
                    signal=signal_from_record(record, now),
                    qr_recent=bool(nested(record, "qr", "visible")),
                    now=now,
                )
            )
            replay = {
                "record_type": "step",
                "time_s": round(local_index * dt_s, 3),
                "dt_s": dt_s,
                "interval_id": interval["interval_id"],
                "profile_name": profile_label,
                "state": output.state,
                "previous_state": previous_state,
                "reason": output.reason,
                "source_log_state": record.get("state"),
                "source_log_reason": record.get("reason"),
                "nav": {
                    "module": module_name,
                    "suggestion_mode": suggestion.mode if suggestion else None,
                    "suggestion_reason": suggestion.reason if suggestion else None,
                    "suggested_linear_x": suggestion.command.linear_x if suggestion else None,
                    "suggested_angular_z": suggestion.command.angular_z if suggestion else None,
                    "debug": suggestion.debug if suggestion else {},
                },
                "arbiter_debug": output.debug,
                "lidar": _sector_record(sectors, 0.0 if lidar_fresh else 999.0, lidar_fresh),
                "freshness": {"lidar_fresh": lidar_fresh, "lidar_age_s": 0.0 if lidar_fresh else 999.0},
                "command": {
                    "requested_linear_x": output.command.linear_x,
                    "requested_angular_z": output.command.angular_z,
                    "published_linear_x": output.command.linear_x,
                    "published_angular_z": output.command.angular_z,
                    "motion_published_to_robot": False,
                },
                "mode_flags": {"dry_run": True, "enable_motion": False, "motion_enabled_this_cycle": False},
                "dry_run": True,
                "enable_motion": False,
            }
            handle.write(json.dumps(_json_clean(replay), ensure_ascii=True, sort_keys=True) + "\n")
            replay_records.append(replay)
            previous_state = output.state

    flags = _record_flags(
        replay_records,
        dt=dt_s,
        front_corner_avoid_distance=float(params.get("front_corner_avoid_distance", 0.62)),
        side_avoid_distance=float(params.get("side_avoid_distance", 0.24)),
        max_yaw=float(params.get("max_yaw", 0.65)),
        spin_yaw_threshold=0.38,
        spin_linear_threshold=0.03,
    )
    count = max(1, len(replay_records))
    turn_tick_count = sum(1 for record in replay_records if record["state"] in TURN_STATES)
    source_has_turn = bool(interval.get("contains_turn"))
    turn_success_proxy = None if not source_has_turn else int(
        bool(replay_records) and replay_records[-1]["state"] == "NAVIGATE" and turn_tick_count > 0
    )
    return {
        "interval_id": interval["interval_id"],
        "profile_name": profile_label,
        "source_log": str(source),
        "replay_path": str(out_path),
        "replay_type": "sector_level",
        "turn_success_proxy": turn_success_proxy,
        "turn_completion_time_s": _duration_in_state(replay_records, "TURNING_LEFT", dt_s)
        + _duration_in_state(replay_records, "TURNING_RIGHT", dt_s)
        + _duration_in_state(replay_records, "TURNING_UTURN", dt_s),
        "recovery_entries_during_turn": _recovery_entries_after_turn(replay_records) if source_has_turn else 0,
        "max_recovery_duration_s": _max_recovery_duration(replay_records, dt_s),
        "recovery_timeout_count": sum(1 for record in replay_records if "TIMEOUT" in str(record["reason"])),
        "front_blocked_select_count": sum(
            1 for record in replay_records if record["reason"] == "FRONT_BLOCKED_SELECT_FREE_GAP"
        ),
        "corner_risk_count": sum(bool(item["corner_risk"]) for item in flags),
        "side_risk_count": sum(bool(item["side_scrape_risk"]) for item in flags),
        "spin_ratio": round(sum(bool(item["spin"]) for item in flags) / count, 4),
        "oscillation_score": round(sum(bool(item["oscillation"]) for item in flags) / count, 4),
        "safety_regression_count": sum(bool(item["corner_risk"]) or bool(item["side_scrape_risk"]) for item in flags),
    }


def _write_csv(path: Path, rows: List[dict]) -> None:
    fields = sorted({key for row in rows for key in row}) or ["interval_id"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_summary(path: Path, rows: List[dict]) -> None:
    lines = [
        "# Turn/Recovery Replay Summary",
        "",
        "Offline sector-level replay only; it does not validate LiDAR angle offset or physical behavior.",
        "",
        "| interval | profile | turn proxy | recovery entries | max recovery s | corner | side | spin | safety regressions |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        proxy = "n/a" if row["turn_success_proxy"] is None else row["turn_success_proxy"]
        lines.append(
            f"| {row['interval_id']} | {row['profile_name']} | {proxy} | {row['recovery_entries_during_turn']} | "
            f"{row['max_recovery_duration_s']:.2f} | {row['corner_risk_count']} | {row['side_risk_count']} | "
            f"{row['spin_ratio']:.4f} | {row['safety_regression_count']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--intervals", type=Path, required=True)
    parser.add_argument("--profiles", nargs="+", default=["wall_follow_tuned"])
    parser.add_argument("--out-dir", type=Path, default=Path("output/turn_recovery_replay"))
    parser.add_argument("--dt", type=float, default=0.1)
    args = parser.parse_args()

    intervals = load_intervals(args.intervals)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = [
        replay_interval(interval, profile, args.out_dir, dt_s=args.dt)
        for interval in intervals
        for profile in args.profiles
    ]
    _write_csv(args.out_dir / "metrics.csv", rows)
    _write_summary(args.out_dir / "summary.md", rows)
    print(f"replayed {len(rows)} interval/profile combinations")
    print("offline sector-level replay only; no /cmd_vel publication")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
