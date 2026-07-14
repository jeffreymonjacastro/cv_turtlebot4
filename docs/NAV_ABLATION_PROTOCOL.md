# Navigation Ablation Protocol

## Purpose

When multiple fixes are added together, it becomes unclear which one helped or hurt.

This protocol requires changes to be tested independently before promotion.

## Candidate fixes

Typical fixes for the current failure modes:

```text
corner-aware yaw veto
corner slowdown
side-risk yaw veto
anti-spin detection
angular smoothing
recovery entry threshold changes
recovery exit threshold changes
follow-gap recovery fallback
wall-follow parameter tuning
```

## Required ablation profiles

For each baseline/tuned candidate, create temporary ablation configs or CLI parameter sets:

```text
baseline
corner_veto_only
corner_slowdown_only
side_veto_only
anti_spin_only
angular_smoothing_only
recovery_changes_only
corner_veto_plus_slowdown
corner_veto_plus_anti_spin
full_candidate
```

Do not permanently promote every ablation profile. Temporary outputs can live under:

```text
output/ablation_runs/
```

## Standard commands

Example:

```bash
python3 scripts/run_nav_ablation.py \
  --base-profile ubuntu/reactive_nav/configs/wall_follow_safe.yaml \
  --scenarios all \
  --real-log output/reactive_nav_debug.jsonl \
  --out-dir output/ablation_runs/wall_follow
```

If `run_nav_ablation.py` does not exist yet, implement it or extend the tuning script with an `--ablation` mode.

## Metrics to compare

Each ablation must report:

```text
pass/warn/fail counts
score
corner_risk_count
side_risk_count
spin_ratio
oscillation_score
yaw_saturation_ratio
recovery_loop_count
emergency_stop_count
commanded_distance_m
average_linear_speed_mps
```

If real logs are available, also report:

```text
real_log_corner_risk_delta
real_log_side_risk_delta
real_log_spin_delta
real_log_oscillation_delta
real_log_yaw_saturation_delta
```

## Decision rules

### Keep

Keep a change when it:

```text
improves one target failure mode
causes no safety regression
causes no material corner/side/spin regression
has a clear explanation in debug fields
```

### Reject

Reject a change when it:

```text
improves average score only by moving faster into unsafe geometry
reduces spin but increases corner risk
reduces corner risk but causes frequent emergency stops in normal corridors
causes unstable oscillation
only helps one synthetic scenario while hurting real-log replay
```

### Investigate

Mark as investigate when:

```text
synthetic metrics improve but real-log replay is neutral
real-log replay improves but synthetic harsh scenarios regress slightly
improvement depends on unrealistic scenario assumptions
```

## Reporting format

The final ablation report should include a compact table:

```text
profile | score | pass/warn/fail | corner | side | spin | oscillation | yaw_sat | recovery | decision
```

And a short decision note:

```text
Promoted: corner_veto_plus_slowdown + angular_smoothing
Rejected: anti_spin_only because it reduced spin but increased recovery loops
Investigate: recovery_changes_only because results differ between synthetic and real-log replay
```

## Promotion

Only export a tuned YAML after the ablation report explains why the selected combination was chosen.
