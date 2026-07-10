#!/usr/bin/env python3
"""Run controlled, offline-only turn/recovery ablations on extracted intervals.

Variants deliberately distinguish applied configuration changes from hypotheses
that need scan-level evidence or a future controller change.  A no-op variant
is never presented as a controller improvement.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import mean
import sys
from typing import Any, Dict, List


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.replay_real_log_nav import _profile_params  # noqa: E402
from scripts.replay_turn_recovery_intervals import load_intervals, replay_interval  # noqa: E402


ABLATIONS = (
    "baseline_current",
    "angle_offset_only",
    "turn_recovery_delay",
    "turn_front_block_tolerance",
    "turn_min_commitment_time",
    "recovery_exit_relaxed",
    "gap_scoring_adjusted",
    "angular_smoothing_adjusted",
    "combined_safe_candidate",
)


def ablation_spec(name: str, base: Dict[str, Any]) -> tuple[Dict[str, Any], str]:
    """Return only behavior changes faithfully representable by sector replay."""
    if name == "baseline_current":
        return {}, "applied_baseline"
    if name == "angle_offset_only":
        return {}, "requires_scan_level_replay"
    if name in {"turn_recovery_delay", "turn_front_block_tolerance", "turn_min_commitment_time"}:
        return {}, "requires_turn_to_recovery_transition_evidence"
    if name == "recovery_exit_relaxed":
        clearance = float(base.get("recovery_clearance", 0.44))
        emergency_clear = float(base.get("front_stop_clear_distance", 0.38))
        return {"recovery_clearance": max(emergency_clear, clearance - 0.04)}, "applied_recovery_clearance_only"
    if name == "gap_scoring_adjusted":
        return {}, "requires_explicit_gap_scoring_parameter"
    if name == "angular_smoothing_adjusted":
        return {"angular_smoothing_alpha": 0.65}, "applied_arbiter_smoothing_only"
    if name == "combined_safe_candidate":
        clearance = float(base.get("recovery_clearance", 0.44))
        emergency_clear = float(base.get("front_stop_clear_distance", 0.38))
        return {
            "recovery_clearance": max(emergency_clear, clearance - 0.04),
            "angular_smoothing_alpha": 0.65,
        }, "applied_only_to_sector_replay_supported_components"
    raise ValueError(f"unknown ablation: {name}")


def _aggregate(name: str, applicability: str, rows: List[dict]) -> dict:
    count = max(1, len(rows))
    proxies = [row["turn_success_proxy"] for row in rows if row["turn_success_proxy"] is not None]
    return {
        "ablation": name,
        "applicability": applicability,
        "interval_count": len(rows),
        "turn_success_proxy": round(mean(proxies), 4) if proxies else None,
        "turn_completion_time_s": round(mean(float(row["turn_completion_time_s"]) for row in rows), 4) if rows else 0.0,
        "recovery_entries_during_turn": sum(int(row["recovery_entries_during_turn"]) for row in rows),
        "max_recovery_duration_s": max((float(row["max_recovery_duration_s"]) for row in rows), default=0.0),
        "recovery_timeout_count": sum(int(row["recovery_timeout_count"]) for row in rows),
        "front_blocked_select_count": sum(int(row["front_blocked_select_count"]) for row in rows),
        "corner_risk_count": sum(int(row["corner_risk_count"]) for row in rows),
        "side_risk_count": sum(int(row["side_risk_count"]) for row in rows),
        "spin_ratio": round(mean(float(row["spin_ratio"]) for row in rows), 4) if rows else 0.0,
        "oscillation_score": round(mean(float(row["oscillation_score"]) for row in rows), 4) if rows else 0.0,
        "safety_regression_count": sum(int(row["safety_regression_count"]) for row in rows),
        "decision": "MEASURE" if applicability.startswith("requires_") else "PENDING_COMPARISON",
    }


def _decision(row: dict, baseline: dict, *, synthetic_safety_passed: bool) -> str:
    if row["applicability"].startswith("requires_"):
        return "MEASURE"
    if row["safety_regression_count"] > baseline["safety_regression_count"]:
        return "REJECT"
    if row["corner_risk_count"] > baseline["corner_risk_count"] or row["side_risk_count"] > baseline["side_risk_count"]:
        return "REJECT"
    if not synthetic_safety_passed:
        # Sector replay is insufficient to promote a robot profile by itself.
        return "MEASURE"
    improved = (
        row["max_recovery_duration_s"] < baseline["max_recovery_duration_s"]
        or row["recovery_entries_during_turn"] < baseline["recovery_entries_during_turn"]
        or row["spin_ratio"] < baseline["spin_ratio"]
    )
    return "KEEP" if improved else "MEASURE"


def _write_csv(path: Path, rows: List[dict]) -> None:
    fields = sorted({key for row in rows for key in row}) or ["ablation"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_summary(path: Path, rows: List[dict], profile: str) -> None:
    lines = [
        "# Turn/Recovery Ablation Summary",
        "",
        "Offline sector-level replay only. No /cmd_vel is published, and angle-offset variants require scan-level data.",
        "",
        f"- profile: `{profile}`",
        "",
        "| ablation | applicability | turn proxy | max recovery s | front selects | corner | side | spin | decision |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        proxy = "n/a" if row["turn_success_proxy"] is None else f"{row['turn_success_proxy']:.3f}"
        lines.append(
            f"| {row['ablation']} | {row['applicability']} | {proxy} | {row['max_recovery_duration_s']:.2f} | "
            f"{row['front_blocked_select_count']} | {row['corner_risk_count']} | {row['side_risk_count']} | "
            f"{row['spin_ratio']:.4f} | {row['decision']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _append_iteration_log(path: Path, rows: List[dict], out_dir: Path) -> None:
    best = next((row for row in rows if row["decision"] == "KEEP"), None)
    entry = [
        "",
        "## Turn/recovery offline ablation",
        "",
        "Hypothesis:",
        "- A targeted turn/recovery replay can identify safe, evidence-supported follow-up changes.",
        "",
        "Change:",
        "- Diagnostics and offline ablation tooling only; no robot motion behavior was promoted by this command.",
        "",
        "Benchmark directories:",
        f"- `{out_dir}`",
        "",
        "Decision:",
        f"- {best['ablation'] if best else 'MEASURE'}; physical validation remains pending.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write("\n".join(entry) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--intervals", type=Path, required=True)
    parser.add_argument("--profile", default="wall_follow_tuned")
    parser.add_argument("--out-dir", type=Path, default=Path("output/turn_recovery_ablation"))
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument(
        "--synthetic-safety-passed",
        action="store_true",
        help="Allow a KEEP decision only after the same variant passed the synthetic safety suite.",
    )
    parser.add_argument("--iteration-log", type=Path, default=Path("output/navigation_iteration_log.md"))
    args = parser.parse_args()

    intervals = load_intervals(args.intervals)
    _module, base = _profile_params(args.profile)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    aggregate: List[dict] = []
    detail: List[dict] = []
    baseline: dict | None = None
    for name in ABLATIONS:
        overrides, applicability = ablation_spec(name, base)
        rows = [
            replay_interval(
                interval,
                args.profile,
                args.out_dir / name,
                dt_s=args.dt,
                params_override=overrides,
                variant_name=name,
            )
            for interval in intervals
        ]
        for row in rows:
            row["ablation"] = name
            row["applicability"] = applicability
        detail.extend(rows)
        summary = _aggregate(name, applicability, rows)
        if name == "baseline_current":
            summary["decision"] = "BASELINE"
            baseline = summary
        else:
            summary["decision"] = _decision(
                summary,
                baseline or summary,
                synthetic_safety_passed=args.synthetic_safety_passed,
            )
        aggregate.append(summary)

    _write_csv(args.out_dir / "metrics.csv", aggregate)
    _write_csv(args.out_dir / "interval_metrics.csv", detail)
    _write_summary(args.out_dir / "summary.md", aggregate, args.profile)
    _append_iteration_log(args.iteration_log, aggregate, args.out_dir)
    print(f"wrote {args.out_dir / 'metrics.csv'}")
    print("offline sector-level ablation only; no /cmd_vel publication")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
