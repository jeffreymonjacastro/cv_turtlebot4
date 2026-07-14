#!/usr/bin/env python3
"""Replay sector-level real/debug logs through current nav profiles.

This script is offline-only. It reconstructs LaserScan-like sector observations
from persistent debug logs, runs the selected navigation module plus arbiter,
and compares old logged command risk against the new simulated command risk.
It never publishes /cmd_vel.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, Iterable, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analyze_robot_failure_log import (  # noqa: E402
    DEFAULT_PATTERNS,
    _record_flags,
    command,
    load_records,
    nested,
    resolve_input_paths,
    sector,
    state,
)
from scripts.compare_nav_profiles import summarize  # noqa: E402
from scripts.replay_nav_scenarios import (  # noqa: E402
    _json_clean,
    _sector_record,
    build_arbiter,
    corridor_scan,
    load_replay_profile,
    nav_kwargs,
)
from ubuntu.reactive_nav.behavior_arbiter import ArbiterInput, SignalState  # noqa: E402
from ubuntu.reactive_nav.lidar_sectors import extract_sectors  # noqa: E402
from ubuntu.reactive_nav.wall_following import NavigationObservation, create_navigation_module  # noqa: E402


PROFILE_CONFIGS = {
    "wall_follow_safe": ("wall_follow", REPO_ROOT / "ubuntu/reactive_nav/configs/wall_follow_safe.yaml"),
    "wall_follow_tuned": ("wall_follow", REPO_ROOT / "ubuntu/reactive_nav/configs/wall_follow_tuned.yaml"),
    "wall_follow_less_conservative": (
        "wall_follow",
        REPO_ROOT / "ubuntu/reactive_nav/configs/wall_follow_less_conservative.yaml",
    ),
    "follow_gap_safe": ("follow_gap", REPO_ROOT / "ubuntu/reactive_nav/configs/follow_gap_safe.yaml"),
    "follow_gap_tuned": ("follow_gap", REPO_ROOT / "ubuntu/reactive_nav/configs/follow_gap_tuned.yaml"),
    "focm_safe": ("focm", REPO_ROOT / "ubuntu/reactive_nav/configs/focm_safe.yaml"),
}


def _safe_filename(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)


def _number(record: dict, *keys: str) -> Optional[float]:
    value = nested(record, *keys)
    if isinstance(value, (int, float)):
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None
    return None


def _sector_or(record: dict, name: str, default: float) -> float:
    value = sector(record, name)
    return default if value is None else value


def scan_from_record(record: dict, t: float):
    return corridor_scan(
        front=_sector_or(record, "front", 1.5),
        front_center=_sector_or(record, "front_center", _sector_or(record, "front", 1.5)),
        front_left=_sector_or(record, "front_left", 1.2),
        front_right=_sector_or(record, "front_right", 1.2),
        left=_sector_or(record, "left", 0.7),
        right=_sector_or(record, "right", 0.7),
        rear=_sector_or(record, "rear", 1.0),
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
        reason=str(payload.get("reason") or "real_log_replay"),
    )


def _original_risk_metrics(records: List[dict], dt: float, max_yaw: float = 0.65) -> Dict[str, Any]:
    flags = _record_flags(
        records,
        dt=dt,
        front_corner_avoid_distance=0.62,
        side_avoid_distance=0.24,
        max_yaw=max_yaw,
        spin_yaw_threshold=0.38,
        spin_linear_threshold=0.03,
    )
    counts = Counter()
    for flag_set in flags:
        for name, active in flag_set.items():
            if active:
                counts[name] += 1
    total = max(1, len(records))
    state_changes = sum(
        1
        for previous, current in zip(records, records[1:])
        if state(previous) != state(current)
    )
    return {
        "old_record_count": len(records),
        "old_corner_risk_count": counts["corner_risk"],
        "old_side_risk_count": counts["side_scrape_risk"],
        "old_spin_tick_count": counts["spin"],
        "old_spin_ratio": round(counts["spin"] / total, 4),
        "old_oscillation_tick_count": counts["oscillation"],
        "old_yaw_saturation_tick_count": counts["yaw_saturation"],
        "old_yaw_saturation_ratio": round(counts["yaw_saturation"] / total, 4),
        "old_recovery_loop_tick_count": counts["recovery_loop"],
        "old_emergency_burst_tick_count": counts["emergency_burst"],
        "old_state_flapping_tick_count": counts["state_flapping"],
        "old_state_transition_count": state_changes,
    }


def _profile_params(profile_name: str) -> tuple[str, Dict[str, Any]]:
    if profile_name in PROFILE_CONFIGS:
        module_name, config_path = PROFILE_CONFIGS[profile_name]
        return module_name, load_replay_profile(module_name, config_path=config_path, profile_name=profile_name)
    module_name = profile_name
    return module_name, load_replay_profile(module_name, profile_name=profile_name)


def replay_file(path: Path, profile_name: str, out_dir: Path, dt_s: float) -> tuple[Path, Dict[str, Any]]:
    records = load_records(path)
    module_name, params = _profile_params(profile_name)
    nav = create_navigation_module(module_name, **nav_kwargs(params))
    arbiter = build_arbiter(params)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{_safe_filename(path.stem)}__{_safe_filename(profile_name)}.jsonl"
    sim_start = time.monotonic()
    previous_state = ""

    with out_path.open("w", encoding="utf-8") as handle:
        metadata = {
            "record_type": "metadata",
            "scenario": f"real_log:{path.stem}",
            "profile_name": profile_name,
            "nav_module": module_name,
            "dt_s": dt_s,
            "duration_s": len(records) * dt_s,
            "source_log": str(path),
            "replay_type": "sector_level_real_log_replay",
            "dry_run": True,
            "enable_motion": False,
            "command_publication": "disabled_offline_replay",
            "config": {key: value for key, value in params.items() if not key.startswith("_")},
        }
        handle.write(json.dumps(_json_clean(metadata), ensure_ascii=True, sort_keys=True) + "\n")

        for index, record in enumerate(records):
            t = index * dt_s
            now = sim_start + t
            scan = scan_from_record(record, t)
            sectors = extract_sectors(
                scan,
                robust_percentile=float(params.get("sector_robust_percentile", 0.10)),
            )
            lidar_fresh_value = nested(record, "freshness", "lidar_fresh")
            lidar_fresh = True if lidar_fresh_value is None else bool(lidar_fresh_value)
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
            original_linear = command(record, "linear_x")
            original_angular = command(record, "angular_z")
            replay_record = {
                "record_type": "step",
                "timestamp": round(t, 3),
                "time_s": round(t, 3),
                "dt_s": dt_s,
                "scenario": f"real_log:{path.stem}",
                "profile_name": profile_name,
                "state": output.state,
                "previous_state": previous_state,
                "reason": output.reason,
                "source_log_state": record.get("state"),
                "source_log_reason": record.get("reason"),
                "source_log_command": {
                    "published_linear_x": original_linear,
                    "published_angular_z": original_angular,
                },
                "command_delta": {
                    "linear_x": output.command.linear_x - original_linear,
                    "angular_z": output.command.angular_z - original_angular,
                },
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
                    "publication_mode": "sector_level_real_log_replay",
                },
                "mode_flags": {
                    "dry_run": True,
                    "enable_motion": False,
                    "motion_enabled_this_cycle": False,
                },
                "dry_run": True,
                "enable_motion": False,
                "collision_event": False,
            }
            handle.write(json.dumps(_json_clean(replay_record), ensure_ascii=True, sort_keys=True) + "\n")
            previous_state = output.state

    summary = summarize(out_path)
    return out_path, summary


def _write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    fieldnames = sorted({key for row in rows for key in row}) or ["source_log", "profile_name"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_markdown(path: Path, rows: List[Dict[str, Any]], profiles: List[str]) -> None:
    lines = [
        "# Real Log Replay Comparison",
        "",
        "Offline sector-level replay only. This does not validate physical robot behavior.",
        "",
        "| source | profile | status | old corner | new corner | old side | new side | old spin | new spin | old yaw sat | new yaw sat | score | decision |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        decision = "keep"
        if int(row.get("new_corner_risk_count") or 0) > int(row.get("old_corner_risk_count") or 0):
            decision = "reject: corner regression"
        elif int(row.get("new_side_risk_count") or 0) > int(row.get("old_side_risk_count") or 0):
            decision = "reject: side regression"
        elif float(row.get("new_spin_ratio") or 0.0) > float(row.get("old_spin_ratio") or 0.0) + 0.05:
            decision = "investigate: spin regression"
        lines.append(
            "| {source_log} | {profile_name} | {new_status} | {old_corner_risk_count} | "
            "{new_corner_risk_count} | {old_side_risk_count} | {new_side_risk_count} | "
            "{old_spin_ratio:.4f} | {new_spin_ratio:.4f} | {old_yaw_saturation_ratio:.4f} | "
            "{new_yaw_saturation_ratio:.4f} | {new_scenario_score:.3f} | "
            f"{decision} |".format(**row)
        )

    tuned_rows = [row for row in rows if row.get("profile_name") == "wall_follow_tuned"]
    if tuned_rows:
        old_corner = sum(int(row.get("old_corner_risk_count") or 0) for row in tuned_rows)
        new_corner = sum(int(row.get("new_corner_risk_count") or 0) for row in tuned_rows)
        old_side = sum(int(row.get("old_side_risk_count") or 0) for row in tuned_rows)
        new_side = sum(int(row.get("new_side_risk_count") or 0) for row in tuned_rows)
        old_spin = sum(float(row.get("old_spin_ratio") or 0.0) for row in tuned_rows) / len(tuned_rows)
        new_spin = sum(float(row.get("new_spin_ratio") or 0.0) for row in tuned_rows) / len(tuned_rows)
        lines.extend(
            [
                "",
                "## wall_follow_tuned Verdict",
                "",
                f"- corner risk: old {old_corner} -> new {new_corner}",
                f"- side risk: old {old_side} -> new {new_side}",
                f"- average spin ratio: old {old_spin:.4f} -> new {new_spin:.4f}",
            ]
        )
    else:
        lines.extend(["", "## wall_follow_tuned Verdict", "", "- Not evaluated in this run."])

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_metric_row(path: Path, profile_name: str, old_metrics: Dict[str, Any], new_summary: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "source_log": path.name,
        "profile_name": profile_name,
        "nav_module": new_summary.get("nav_module"),
        "new_status": new_summary.get("status"),
        "new_scenario_score": float(new_summary.get("scenario_score") or 0.0),
        "old_corner_risk_count": int(old_metrics.get("old_corner_risk_count") or 0),
        "new_corner_risk_count": int(new_summary.get("corner_risk_count") or 0),
        "corner_risk_delta": int(new_summary.get("corner_risk_count") or 0) - int(old_metrics.get("old_corner_risk_count") or 0),
        "old_side_risk_count": int(old_metrics.get("old_side_risk_count") or 0),
        "new_side_risk_count": int(new_summary.get("side_risk_count") or 0),
        "side_risk_delta": int(new_summary.get("side_risk_count") or 0) - int(old_metrics.get("old_side_risk_count") or 0),
        "old_spin_ratio": float(old_metrics.get("old_spin_ratio") or 0.0),
        "new_spin_ratio": float(new_summary.get("spin_ratio") or 0.0),
        "spin_ratio_delta": float(new_summary.get("spin_ratio") or 0.0) - float(old_metrics.get("old_spin_ratio") or 0.0),
        "old_yaw_saturation_ratio": float(old_metrics.get("old_yaw_saturation_ratio") or 0.0),
        "new_yaw_saturation_ratio": float(new_summary.get("yaw_saturation_ratio") or 0.0),
        "yaw_saturation_delta": float(new_summary.get("yaw_saturation_ratio") or 0.0)
        - float(old_metrics.get("old_yaw_saturation_ratio") or 0.0),
        "new_oscillation_score": float(new_summary.get("oscillation_score") or 0.0),
        "new_recovery_loop_count": int(new_summary.get("recovery_loop_count") or 0),
        "new_average_linear_speed_mps": float(new_summary.get("average_published_linear_speed_mps") or 0.0),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path)
    parser.add_argument("--profiles", nargs="+", default=["wall_follow_safe", "wall_follow_tuned", "follow_gap_safe"])
    parser.add_argument("--out-dir", type=Path, default=Path("output/real_log_replay"))
    parser.add_argument("--pattern", action="append", dest="patterns")
    parser.add_argument("--dt", type=float, default=0.1)
    args = parser.parse_args()

    paths, searched = resolve_input_paths(args.paths, tuple(args.patterns or DEFAULT_PATTERNS))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not paths:
        (args.out_dir / "comparison.md").write_text(
            "# Real Log Replay Comparison\n\n"
            "No real/debug logs found. Searched:\n\n"
            + "\n".join(f"- `{item}`" for item in searched)
            + "\n",
            encoding="utf-8",
        )
        print(f"no real/debug logs found; searched: {', '.join(searched)}")
        return 0

    rows: List[Dict[str, Any]] = []
    written: List[Path] = []
    for path in paths:
        records = load_records(path)
        old_metrics = _original_risk_metrics(records, args.dt)
        for profile_name in args.profiles:
            out_path, new_summary = replay_file(path, profile_name, args.out_dir, args.dt)
            written.append(out_path)
            rows.append(build_metric_row(path, profile_name, old_metrics, new_summary))

    metrics_csv = args.out_dir / "metrics.csv"
    comparison_md = args.out_dir / "comparison.md"
    _write_csv(metrics_csv, rows)
    _write_markdown(comparison_md, rows, args.profiles)
    for path in written:
        print(f"wrote {path}")
    print(f"wrote {metrics_csv}")
    print(f"wrote {comparison_md}")
    print("sector-level real-log replay only; not physical validation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
