#!/usr/bin/env python3
"""Run controlled offline ablations for reactive navigation fixes.

The ablations are intentionally config/toggle based. They reuse the normal
synthetic replay path and optional sector-level real-log replay, and they never
publish /cmd_vel.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
import json
from pathlib import Path
from statistics import mean
import sys
import time
from typing import Any, Dict, Iterable, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analyze_robot_failure_log import DEFAULT_PATTERNS, load_records, resolve_input_paths  # noqa: E402
from scripts.compare_nav_profiles import summarize  # noqa: E402
from scripts.replay_nav_scenarios import (  # noqa: E402
    _json_clean,
    _sector_record,
    build_arbiter,
    load_replay_profile,
    nav_kwargs,
    resolve_scenarios,
    run_scenario,
)
from scripts.replay_real_log_nav import (  # noqa: E402
    _original_risk_metrics,
    scan_from_record,
    signal_from_record,
)
from ubuntu.reactive_nav.behavior_arbiter import ArbiterInput  # noqa: E402
from ubuntu.reactive_nav.lidar_sectors import extract_sectors  # noqa: E402
from ubuntu.reactive_nav.reactive_navigator import load_profile_parameters  # noqa: E402
from ubuntu.reactive_nav.wall_following import NavigationObservation, create_navigation_module  # noqa: E402


ABLATION_NAMES = (
    "baseline",
    "corner_veto_only",
    "corner_slowdown_only",
    "side_veto_only",
    "anti_spin_only",
    "angular_smoothing_only",
    "recovery_changes_only",
    "corner_veto_plus_slowdown",
    "corner_veto_plus_anti_spin",
    "full_candidate",
)


def _safe_filename(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)


def _load_base_params(path: Path) -> Dict[str, Any]:
    params = load_replay_profile("wall_follow", config_path=path, profile_name=path.stem)
    params.update(load_profile_parameters(path))
    params["nav_module"] = str(params.get("nav_module") or "wall_follow")
    params["_config_path"] = str(path)
    return params


def _neutralize(params: Dict[str, Any]) -> Dict[str, Any]:
    neutral = dict(params)
    neutral.update(
        {
            "enable_corner_yaw_veto": False,
            "enable_corner_slowdown": False,
            "enable_side_yaw_veto": False,
            "enable_anti_spin": False,
            "angular_smoothing_alpha": 1.0,
        }
    )
    return neutral


def ablation_params(name: str, base: Dict[str, Any]) -> Dict[str, Any]:
    params = _neutralize(base)
    params["profile_name"] = name
    params["ablation_name"] = name

    if name == "baseline":
        return params
    if name == "corner_veto_only":
        params["enable_corner_yaw_veto"] = True
        return params
    if name == "corner_slowdown_only":
        params["enable_corner_slowdown"] = True
        return params
    if name == "side_veto_only":
        params["enable_side_yaw_veto"] = True
        return params
    if name == "anti_spin_only":
        params["enable_anti_spin"] = True
        params["anti_spin_trigger_cycles"] = 4
        params["anti_spin_yaw_threshold"] = min(0.42, float(base.get("max_yaw", 0.65)) * 0.55)
        return params
    if name == "angular_smoothing_only":
        params["angular_smoothing_alpha"] = 0.55
        return params
    if name == "recovery_changes_only":
        params["front_clear_distance"] = max(float(base.get("front_clear_distance", 0.58)), 0.64)
        params["recovery_clearance"] = max(float(base.get("recovery_clearance", 0.44)), 0.48)
        params["narrow_speed"] = min(float(base.get("narrow_speed", 0.05)), 0.035)
        return params
    if name == "corner_veto_plus_slowdown":
        params["enable_corner_yaw_veto"] = True
        params["enable_corner_slowdown"] = True
        return params
    if name == "corner_veto_plus_anti_spin":
        params["enable_corner_yaw_veto"] = True
        params["enable_anti_spin"] = True
        params["anti_spin_trigger_cycles"] = 4
        params["anti_spin_yaw_threshold"] = min(0.42, float(base.get("max_yaw", 0.65)) * 0.55)
        return params
    if name == "full_candidate":
        params = dict(base)
        params.update(
            {
                "profile_name": name,
                "ablation_name": name,
                "enable_corner_yaw_veto": True,
                "enable_corner_slowdown": True,
                "enable_side_yaw_veto": True,
                "enable_anti_spin": True,
                "anti_spin_trigger_cycles": 4,
                "anti_spin_yaw_threshold": min(0.42, float(base.get("max_yaw", 0.65)) * 0.55),
                "angular_smoothing_alpha": 0.65,
                "front_clear_distance": max(float(base.get("front_clear_distance", 0.58)), 0.64),
                "recovery_clearance": max(float(base.get("recovery_clearance", 0.44)), 0.48),
            }
        )
        return params
    raise ValueError(f"unknown ablation: {name}")


def _aggregate_summaries(name: str, summaries: List[Dict[str, Any]]) -> Dict[str, Any]:
    total_score = sum(float(summary.get("scenario_score") or 0.0) for summary in summaries)
    count = max(1, len(summaries))
    statuses = Counter(str(summary.get("status")) for summary in summaries)
    return {
        "profile": name,
        "scenario_count": len(summaries),
        "pass_count": statuses["PASS"],
        "warn_count": statuses["WARN"],
        "fail_count": statuses["FAIL"],
        "total_score": round(total_score, 3),
        "avg_score": round(total_score / count, 3),
        "corner_risk_count": sum(int(summary.get("corner_risk_count") or 0) for summary in summaries),
        "side_risk_count": sum(int(summary.get("side_risk_count") or 0) for summary in summaries),
        "avg_spin_ratio": round(mean(float(summary.get("spin_ratio") or 0.0) for summary in summaries), 4),
        "avg_oscillation_score": round(mean(float(summary.get("oscillation_score") or 0.0) for summary in summaries), 3),
        "avg_yaw_saturation_ratio": round(mean(float(summary.get("yaw_saturation_ratio") or 0.0) for summary in summaries), 4),
        "recovery_loop_count": sum(int(summary.get("recovery_loop_count") or 0) for summary in summaries),
        "emergency_stop_count": sum(int(summary.get("emergency_stop_count") or 0) for summary in summaries),
        "commanded_distance_m": round(sum(float(summary.get("commanded_distance_estimate_m") or 0.0) for summary in summaries), 4),
        "avg_linear_speed_mps": round(mean(float(summary.get("average_published_linear_speed_mps") or 0.0) for summary in summaries), 4),
    }


def _replay_real_log_with_params(
    *,
    path: Path,
    params: Dict[str, Any],
    out_dir: Path,
    dt_s: float,
) -> Dict[str, Any]:
    records = load_records(path)
    module_name = str(params.get("nav_module") or "wall_follow")
    nav = create_navigation_module(module_name, **nav_kwargs(params))
    arbiter = build_arbiter(params)
    profile = str(params.get("profile_name") or "ablation")
    out_path = out_dir / f"{_safe_filename(path.stem)}__{_safe_filename(profile)}.jsonl"
    sim_start = time.monotonic()
    previous_state = ""

    with out_path.open("w", encoding="utf-8") as handle:
        handle.write(
            json.dumps(
                _json_clean(
                    {
                        "record_type": "metadata",
                        "scenario": f"real_log:{path.stem}",
                        "profile_name": profile,
                        "nav_module": module_name,
                        "dt_s": dt_s,
                        "duration_s": len(records) * dt_s,
                        "source_log": str(path),
                        "replay_type": "ablation_sector_level_real_log_replay",
                        "dry_run": True,
                        "enable_motion": False,
                        "config": {key: value for key, value in params.items() if not key.startswith("_")},
                    }
                ),
                ensure_ascii=True,
                sort_keys=True,
            )
            + "\n"
        )
        for index, record in enumerate(records):
            t = index * dt_s
            now = sim_start + t
            sectors = extract_sectors(
                scan_from_record(record, t),
                robust_percentile=float(params.get("sector_robust_percentile", 0.10)),
            )
            lidar_fresh = bool(record.get("freshness", {}).get("lidar_fresh", True))
            nav_suggestion = nav.compute(NavigationObservation(sectors, now, dt_s)) if lidar_fresh else None
            output = arbiter.decide(
                ArbiterInput(
                    sectors=sectors,
                    lidar_fresh=lidar_fresh,
                    nav_suggestion=nav_suggestion,
                    signal=signal_from_record(record, now),
                    qr_recent=bool(record.get("qr", {}).get("visible")),
                    now=now,
                )
            )
            replay_record = {
                "record_type": "step",
                "timestamp": round(t, 3),
                "time_s": round(t, 3),
                "dt_s": dt_s,
                "scenario": f"real_log:{path.stem}",
                "profile_name": profile,
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
                "command": {
                    "requested_linear_x": output.command.linear_x,
                    "requested_angular_z": output.command.angular_z,
                    "published_linear_x": output.command.linear_x,
                    "published_angular_z": output.command.angular_z,
                    "motion_published_to_robot": False,
                    "publication_mode": "ablation_sector_level_real_log_replay",
                },
                "dry_run": True,
                "enable_motion": False,
            }
            handle.write(json.dumps(_json_clean(replay_record), ensure_ascii=True, sort_keys=True) + "\n")
            previous_state = output.state
    summary = summarize(out_path)
    old = _original_risk_metrics(records, dt_s)
    return {
        "real_log_path": str(path),
        "real_log_output": str(out_path),
        "old_corner_risk_count": int(old.get("old_corner_risk_count") or 0),
        "new_corner_risk_count": int(summary.get("corner_risk_count") or 0),
        "old_side_risk_count": int(old.get("old_side_risk_count") or 0),
        "new_side_risk_count": int(summary.get("side_risk_count") or 0),
        "old_spin_ratio": float(old.get("old_spin_ratio") or 0.0),
        "new_spin_ratio": float(summary.get("spin_ratio") or 0.0),
        "old_yaw_saturation_ratio": float(old.get("old_yaw_saturation_ratio") or 0.0),
        "new_yaw_saturation_ratio": float(summary.get("yaw_saturation_ratio") or 0.0),
    }


def _real_log_aggregate(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not rows:
        return {
            "real_log_count": 0,
            "real_log_corner_risk_delta": 0,
            "real_log_side_risk_delta": 0,
            "real_log_spin_delta": 0.0,
            "real_log_yaw_saturation_delta": 0.0,
        }
    return {
        "real_log_count": len(rows),
        "real_log_corner_risk_delta": sum(row["new_corner_risk_count"] - row["old_corner_risk_count"] for row in rows),
        "real_log_side_risk_delta": sum(row["new_side_risk_count"] - row["old_side_risk_count"] for row in rows),
        "real_log_spin_delta": round(mean(row["new_spin_ratio"] - row["old_spin_ratio"] for row in rows), 4),
        "real_log_yaw_saturation_delta": round(
            mean(row["new_yaw_saturation_ratio"] - row["old_yaw_saturation_ratio"] for row in rows),
            4,
        ),
    }


def _decision(row: Dict[str, Any], baseline: Dict[str, Any]) -> str:
    if int(row["fail_count"]) > 0 or int(row["corner_risk_count"]) > 0 or int(row["side_risk_count"]) > 0:
        return "REJECT"
    if float(row["real_log_corner_risk_delta"]) > 0 or float(row["real_log_side_risk_delta"]) > 0:
        return "REJECT"
    if float(row["avg_score"]) > float(baseline["avg_score"]) and float(row["avg_spin_ratio"]) <= float(baseline["avg_spin_ratio"]) + 0.05:
        return "KEEP"
    if float(row["real_log_spin_delta"]) < -0.02 or float(row["avg_spin_ratio"]) < float(baseline["avg_spin_ratio"]):
        return "INVESTIGATE"
    return "MEASURE"


def _write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    fieldnames = sorted({key for row in rows for key in row}) or ["profile"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_summary(path: Path, rows: List[Dict[str, Any]], base_profile: Path, real_logs: List[Path]) -> None:
    lines = [
        "# Navigation Ablation Summary",
        "",
        "Offline validation only. No Gazebo, no TurtleBot, no /cmd_vel publication.",
        "",
        f"- base profile: `{base_profile}`",
        f"- real logs used: {len(real_logs)}",
        "",
        "| profile | score | pass/warn/fail | corner | side | spin | oscillation | yaw sat | recovery | real corner delta | real spin delta | decision |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        lines.append(
            "| {profile} | {avg_score:.3f} | {pass_count}/{warn_count}/{fail_count} | "
            "{corner_risk_count} | {side_risk_count} | {avg_spin_ratio:.4f} | "
            "{avg_oscillation_score:.3f} | {avg_yaw_saturation_ratio:.4f} | "
            "{recovery_loop_count} | {real_log_corner_risk_delta} | "
            "{real_log_spin_delta:.4f} | {decision} |".format(**row)
        )

    keeps = [row["profile"] for row in rows if row.get("decision") == "KEEP"]
    rejects = [row["profile"] for row in rows if row.get("decision") == "REJECT"]
    investigate = [row["profile"] for row in rows if row.get("decision") == "INVESTIGATE"]
    lines.extend(
        [
            "",
            "## Decision Notes",
            "",
            f"- Keep: {', '.join(keeps) if keeps else 'none'}",
            f"- Reject: {', '.join(rejects) if rejects else 'none'}",
            f"- Investigate: {', '.join(investigate) if investigate else 'none'}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-profile", type=Path, default=REPO_ROOT / "ubuntu/reactive_nav/configs/wall_follow_tuned.yaml")
    parser.add_argument("--scenarios", nargs="+", default=["all"])
    parser.add_argument("--out-dir", type=Path, default=Path("output/ablation_runs/real_log_iter"))
    parser.add_argument("--real-log", nargs="*", type=Path)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=11)
    args = parser.parse_args()

    base = _load_base_params(args.base_profile)
    module_name = str(base.get("nav_module") or "wall_follow")
    scenarios = resolve_scenarios(args.scenarios, args.duration_s)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    if args.real_log is None:
        real_logs, _searched = resolve_input_paths([], DEFAULT_PATTERNS)
    else:
        real_logs, _searched = resolve_input_paths(args.real_log, DEFAULT_PATTERNS)

    scenario_rows: List[Dict[str, Any]] = []
    aggregate_rows: List[Dict[str, Any]] = []
    baseline_row: Optional[Dict[str, Any]] = None

    for name in ABLATION_NAMES:
        params = ablation_params(name, base)
        summaries = []
        for scenario in scenarios:
            path = run_scenario(
                scenario=scenario,
                module_name=module_name,
                params=params,
                out_dir=args.out_dir,
                seed=args.seed,
                dt_s=args.dt,
                duration_s=scenario.duration_s,
            )
            summary = summarize(path)
            summary["ablation"] = name
            summaries.append(summary)
            scenario_rows.append(summary)

        row = _aggregate_summaries(name, summaries)
        real_rows = [
            _replay_real_log_with_params(path=path, params=params, out_dir=args.out_dir, dt_s=args.dt)
            for path in real_logs
        ]
        row.update(_real_log_aggregate(real_rows))
        if name == "baseline":
            baseline_row = row
            row["decision"] = "BASELINE"
        else:
            row["decision"] = _decision(row, baseline_row or row)
        aggregate_rows.append(row)

    metrics_csv = args.out_dir / "metrics.csv"
    scenario_csv = args.out_dir / "scenario_metrics.csv"
    summary_md = args.out_dir / "summary.md"
    _write_csv(metrics_csv, aggregate_rows)
    _write_csv(scenario_csv, scenario_rows)
    _write_summary(summary_md, aggregate_rows, args.base_profile, real_logs)
    print(f"wrote {metrics_csv}")
    print(f"wrote {scenario_csv}")
    print(f"wrote {summary_md}")
    print("offline ablation only; not physical validation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
