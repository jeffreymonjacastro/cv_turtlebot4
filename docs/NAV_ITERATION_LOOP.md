# NAV_ITERATION_LOOP.md — Benchmark-Driven Navigation Improvement Protocol

## Goal

Navigation development must follow a controlled loop:

```text
benchmark -> identify problem -> tweak -> benchmark -> decide
```

This prevents random tuning and makes it possible to improve the system without repeatedly using the physical TurtleBot.

## Non-negotiable rule

Do not make several unrelated behavior changes at once.

Prefer one small controlled change, then measure whether it helped.

Examples of good changes:

- add corner-aware yaw veto
- add angular smoothing
- reduce `max_yaw` in one profile
- add anti-spin transition to recovery
- adjust recovery exit condition
- add one new scenario exposing a known failure

Examples of bad changes:

- rewrite all navigation modules
- tune many parameters manually without recording before/after metrics
- increase speed and yaw to improve progress while increasing corner risk
- remove emergency stops to improve score
- declare success from one metric only

## Loop stages

### Stage 1 — Baseline

Run the current benchmark without code changes.

Suggested commands:

```bash
python3 -m pytest tests/

python3 scripts/replay_nav_scenarios.py \
  --nav-modules wall_follow follow_gap focm \
  --scenarios all \
  --out-dir output/iter_baseline

python3 scripts/compare_nav_profiles.py \
  output/iter_baseline/*.jsonl \
  --summary-csv output/iter_baseline/summary.csv \
  --summary-md output/iter_baseline/summary.md
```

If the exact CLI differs, use the existing scripts and preserve equivalent outputs.

### Stage 2 — Identify the dominant weakness

Inspect metrics and logs. Choose one target weakness.

Priority order:

1. Collision/corner/side risk
2. stale/invalid sensor safety
3. spinning/circling
4. oscillation
5. recovery loops
6. low progress
7. score improvement

Safety dominates progress.

### Stage 3 — Create or select a regression scenario

If a real failure is not represented by a scenario, add one before tuning.

Example:

```text
Observed: robot hits front-left corners.
Action: add `front_left_corner_blocked` scenario.
Expected: yaw into left corner is vetoed or strongly penalized.
```

### Stage 4 — Make one change

Implement one controlled fix or parameter adjustment.

Record the hypothesis in code comments or the summary:

```text
Hypothesis: corner-aware yaw veto reduces front-left/front-right risk without increasing spin ratio.
```

### Stage 5 — Re-run benchmark

Use the same command as baseline but a different output directory:

```bash
python3 scripts/replay_nav_scenarios.py \
  --nav-modules wall_follow follow_gap focm \
  --scenarios all \
  --out-dir output/iter_after_change_001

python3 scripts/compare_nav_profiles.py \
  output/iter_after_change_001/*.jsonl \
  --baseline output/iter_baseline/summary.csv \
  --summary-csv output/iter_after_change_001/summary.csv \
  --summary-md output/iter_after_change_001/summary.md
```

### Stage 6 — Decide

Keep the change only if it improves the targeted weakness without unacceptable regressions.

Decision categories:

```text
KEEP        improves target, safety preserved
REJECT      worsens safety or core behavior
ADJUST      promising but needs parameter refinement
SPLIT       change mixes multiple ideas; split into smaller changes
MEASURE     metrics inconclusive; add better scenario/metric first
```

### Stage 7 — Repeat

Continue until the benchmark reveals no obvious low-cost improvements or the time budget is reached.

## Parameter tuning protocol

Use bounded deterministic search, not open-ended tuning.

Recommended modes:

### Manual profile comparison

Create a small set of named configs:

```text
wall_follow_safe.yaml
wall_follow_corner_slow.yaml
wall_follow_low_yaw.yaml
wall_follow_smooth.yaml
follow_gap_recovery_safe.yaml
```

Benchmark all.

### Random search with fixed seed

Use when parameters interact.

Example:

```bash
python3 scripts/tune_nav_profiles.py \
  --nav-modules wall_follow follow_gap \
  --scenarios all \
  --trials 100 \
  --seed 7 \
  --out-dir output/tuning_runs
```

The tuner must:

- save every candidate config
- save every candidate metric row
- reject unsafe candidates
- rank safe candidates
- export only promoted configs

## Suggested parameter ranges

### wall_follow

```text
base_speed:                 0.035 to 0.12
narrow_speed:               0.020 to 0.070
corner_speed:               0.015 to 0.050
max_yaw:                    0.35 to 0.80
wall_kp:                    0.25 to 0.80
wall_kd:                    0.00 to 0.08
front_stop_distance:        0.28 to 0.40
front_slow_distance:        0.45 to 0.75
front_corner_avoid_distance:0.40 to 0.80
side_avoid_distance:        0.22 to 0.45
avoidance_gain:             0.40 to 1.20
max_angular_delta_per_cycle:0.04 to 0.18
spin_detection_window_s:    1.0 to 3.5
spin_yaw_threshold:         0.35 to 0.70
spin_linear_threshold:      0.00 to 0.04
```

### follow_gap

```text
base_speed:                 0.025 to 0.10
max_yaw:                    0.35 to 0.85
gap_bubble_radius_m:        0.20 to 0.45
gap_min_width_deg:          12.0 to 35.0
gap_side_margin_m:          0.04 to 0.16
front_stop_distance:        0.28 to 0.42
front_corner_avoid_distance:0.40 to 0.80
```

### focm

FOCM remains experimental unless it clearly improves.

```text
focm_alpha:                 20.0 to 70.0
robot_width_m:              0.34 to 0.42
gap_side_margin_m:          0.04 to 0.16
gap_min_width_deg:          12.0 to 35.0
base_speed:                 0.025 to 0.08
max_yaw:                    0.35 to 0.75
```

## Change log format

Every iterative tuning run should append a short entry to:

```text
output/navigation_iteration_log.md
```

Format:

```md
## Iteration N — <short name>

Hypothesis:
- ...

Change:
- ...

Benchmark:
- baseline dir: ...
- after dir: ...

Improved:
- ...

Regressed:
- ...

Decision:
- KEEP / REJECT / ADJUST / SPLIT / MEASURE

Notes:
- offline validation only
```

## Stop conditions

Stop the loop when:

- safety regressions appear and cannot be resolved quickly
- benchmark no longer reflects the observed robot failures
- all improvements are overfitting to synthetic scenarios
- changes become too broad to evaluate cleanly
- time budget is reached

At that point, preserve the best configs and provide next physical dry-run instructions.
