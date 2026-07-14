#!/usr/bin/env python3
"""Bounded deterministic tuning for offline reactive-nav profiles."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import random
import sys
from typing import Any, Dict, Iterable, List


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.compare_nav_profiles import summarize  # noqa: E402
from scripts.replay_nav_scenarios import (  # noqa: E402
    load_replay_profile,
    resolve_scenarios,
    run_scenario,
)


CONFIG_DIR = REPO_ROOT / "ubuntu/reactive_nav/configs"


def _sample_uniform(rng: random.Random, low: float, high: float, digits: int = 4) -> float:
    return round(rng.uniform(low, high), digits)


def sample_candidate(module_name: str, rng: random.Random, base: Dict[str, Any]) -> Dict[str, Any]:
    candidate = dict(base)
    if module_name == "wall_follow":
        candidate.update(
            {
                "base_speed": _sample_uniform(rng, 0.035, 0.10),
                "narrow_speed": _sample_uniform(rng, 0.020, 0.060),
                "turn_slow_speed": _sample_uniform(rng, 0.020, 0.060),
                "corner_slow_speed": _sample_uniform(rng, 0.015, 0.045),
                "max_yaw": _sample_uniform(rng, 0.35, 0.68),
                "wall_kp": _sample_uniform(rng, 0.25, 0.70),
                "wall_kd": _sample_uniform(rng, 0.00, 0.07),
                "front_clear_distance": _sample_uniform(rng, 0.52, 0.72),
                "slow_distance": _sample_uniform(rng, 0.48, 0.70),
                "front_corner_avoid_distance": _sample_uniform(rng, 0.54, 0.78),
                "side_avoid_distance": _sample_uniform(rng, 0.26, 0.42),
                "avoidance_gain": _sample_uniform(rng, 0.55, 1.10),
            }
        )
    elif module_name == "follow_gap":
        candidate.update(
            {
                "base_speed": _sample_uniform(rng, 0.025, 0.085),
                "narrow_speed": _sample_uniform(rng, 0.020, 0.055),
                "turn_slow_speed": _sample_uniform(rng, 0.020, 0.055),
                "corner_slow_speed": _sample_uniform(rng, 0.015, 0.045),
                "max_yaw": _sample_uniform(rng, 0.35, 0.72),
                "front_clear_distance": _sample_uniform(rng, 0.52, 0.72),
                "slow_distance": _sample_uniform(rng, 0.50, 0.72),
                "front_corner_avoid_distance": _sample_uniform(rng, 0.54, 0.78),
                "gap_bubble_radius_m": _sample_uniform(rng, 0.20, 0.42),
                "gap_min_width_deg": _sample_uniform(rng, 14.0, 32.0),
                "gap_side_margin_m": _sample_uniform(rng, 0.04, 0.14),
                "gap_heading_scale_deg": _sample_uniform(rng, 68.0, 95.0),
            }
        )
    else:
        raise ValueError(f"Unsupported tuning module: {module_name}")

    candidate["profile_name"] = f"{module_name}_candidate"
    candidate["nav_module"] = module_name
    candidate["dry_run"] = True
    candidate["enable_motion"] = False
    return candidate


def _candidate_is_safe(summaries: Iterable[Dict[str, Any]]) -> bool:
    for summary in summaries:
        if summary.get("status") == "FAIL":
            return False
        if int(summary.get("corner_risk_count") or 0) > 0:
            return False
        if int(summary.get("side_risk_count") or 0) > 0:
            return False
        if int(summary.get("unsafe_forward_while_blocked_count") or 0) > 0:
            return False
        if int(summary.get("stale_lidar_motion_violation_count") or 0) > 0:
            return False
        if int(summary.get("invalid_lidar_motion_violation_count") or 0) > 0:
            return False
    return True


def _aggregate(module_name: str, trial_id: str, summaries: List[Dict[str, Any]]) -> Dict[str, Any]:
    score = sum(float(summary.get("scenario_score") or 0.0) for summary in summaries)
    return {
        "trial_id": trial_id,
        "nav_module": module_name,
        "scenario_count": len(summaries),
        "total_score": round(score, 3),
        "avg_score": round(score / max(1, len(summaries)), 3),
        "pass_count": sum(summary.get("status") == "PASS" for summary in summaries),
        "warn_count": sum(summary.get("status") == "WARN" for summary in summaries),
        "fail_count": sum(summary.get("status") == "FAIL" for summary in summaries),
        "corner_risk_count": sum(int(summary.get("corner_risk_count") or 0) for summary in summaries),
        "side_risk_count": sum(int(summary.get("side_risk_count") or 0) for summary in summaries),
        "unsafe_forward_while_blocked_count": sum(int(summary.get("unsafe_forward_while_blocked_count") or 0) for summary in summaries),
        "stale_lidar_motion_violation_count": sum(int(summary.get("stale_lidar_motion_violation_count") or 0) for summary in summaries),
        "invalid_lidar_motion_violation_count": sum(int(summary.get("invalid_lidar_motion_violation_count") or 0) for summary in summaries),
        "safe": _candidate_is_safe(summaries),
    }


def _write_yaml(path: Path, params: Dict[str, Any]) -> None:
    keep_keys = [
        "profile_name",
        "nav_module",
        "base_speed",
        "narrow_speed",
        "turn_slow_speed",
        "corner_slow_speed",
        "turn_slow_yaw_threshold",
        "max_yaw",
        "wall_kp",
        "wall_kd",
        "front_clear_distance",
        "slow_distance",
        "recovery_clearance",
        "side_avoid_distance",
        "front_corner_avoid_distance",
        "front_stop_distance",
        "front_stop_clear_distance",
        "side_stop_distance",
        "side_stop_clear_distance",
        "avoidance_gain",
        "gap_bubble_radius_m",
        "gap_min_width_deg",
        "gap_search_min_deg",
        "gap_search_max_deg",
        "gap_heading_scale_deg",
        "gap_distance_score_cap_m",
        "gap_forward_cone_deg",
        "robot_width_m",
        "gap_side_margin_m",
        "turn_clearance",
    ]
    lines = ["/**:", "  ros__parameters:"]
    for key in keep_keys:
        if key not in params:
            continue
        value = params[key]
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        else:
            rendered = str(value)
        lines.append(f"    {key}: {rendered}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def evaluate_candidate(
    *,
    module_name: str,
    params: Dict[str, Any],
    scenarios,
    out_dir: Path,
    trial_id: str,
    seed: int,
    dt_s: float,
) -> List[Dict[str, Any]]:
    run_dir = out_dir / "runs" / module_name / trial_id
    summaries = []
    for scenario in scenarios:
        path = run_scenario(
            scenario=scenario,
            module_name=module_name,
            params=params,
            out_dir=run_dir,
            seed=seed,
            dt_s=dt_s,
            duration_s=scenario.duration_s,
        )
        summaries.append(summarize(path))
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav-modules", nargs="+", default=["wall_follow", "follow_gap"])
    parser.add_argument("--scenarios", nargs="+", default=["all"])
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--out-dir", type=Path, default=Path("output/tuning_runs"))
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--no-export", action="store_true", help="Do not export winning configs")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    scenarios = resolve_scenarios(args.scenarios, args.duration_s)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    candidates_dir = args.out_dir / "candidates"
    candidates_dir.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, Any]] = []
    best_by_module: Dict[str, tuple[Dict[str, Any], Dict[str, Any]]] = {}

    for module_name in args.nav_modules:
        if module_name not in {"wall_follow", "follow_gap"}:
            print(f"skip unsupported tuning module={module_name}", file=sys.stderr)
            continue

        base = load_replay_profile(module_name)
        base["profile_name"] = f"{module_name}_baseline"
        base_summaries = evaluate_candidate(
            module_name=module_name,
            params=base,
            scenarios=scenarios,
            out_dir=args.out_dir,
            trial_id="baseline",
            seed=args.seed,
            dt_s=args.dt,
        )
        baseline_row = _aggregate(module_name, "baseline", base_summaries)
        baseline_row["exported"] = False
        rows.append(baseline_row)

        for trial in range(args.trials):
            trial_id = f"trial_{trial:04d}"
            candidate = sample_candidate(module_name, rng, base)
            candidate["profile_name"] = f"{module_name}_{trial_id}"
            candidate_path = candidates_dir / f"{module_name}_{trial_id}.yaml"
            _write_yaml(candidate_path, candidate)
            summaries = evaluate_candidate(
                module_name=module_name,
                params=candidate,
                scenarios=scenarios,
                out_dir=args.out_dir,
                trial_id=trial_id,
                seed=args.seed + trial + 1,
                dt_s=args.dt,
            )
            row = _aggregate(module_name, trial_id, summaries)
            row["candidate_path"] = str(candidate_path)
            row["exported"] = False
            rows.append(row)
            if row["safe"] and row["total_score"] > baseline_row["total_score"]:
                current_best = best_by_module.get(module_name)
                if current_best is None or row["total_score"] > current_best[0]["total_score"]:
                    best_by_module[module_name] = (row, candidate)

    for module_name, (row, params) in best_by_module.items():
        if args.no_export:
            continue
        export_path = CONFIG_DIR / f"{module_name}_tuned.yaml"
        params = dict(params)
        params["profile_name"] = f"{module_name}_tuned"
        _write_yaml(export_path, params)
        row["exported"] = True
        row["export_path"] = str(export_path)

    results_json = args.out_dir / "tuning_results.json"
    results_csv = args.out_dir / "tuning_results.csv"
    results_json.write_text(json.dumps(rows, indent=2, sort_keys=True), encoding="utf-8")
    fieldnames = sorted({key for row in rows for key in row})
    with results_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {results_csv}")
    print(f"wrote {results_json}")
    for module_name in args.nav_modules:
        safe_rows = [row for row in rows if row.get("nav_module") == module_name and row.get("safe")]
        if not safe_rows:
            print(f"{module_name}: no safe candidate beat baseline")
            continue
        best = max(safe_rows, key=lambda row: float(row["total_score"]))
        print(
            f"{module_name}: best={best['trial_id']} score={best['total_score']} "
            f"safe={best['safe']} exported={best.get('exported', False)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
