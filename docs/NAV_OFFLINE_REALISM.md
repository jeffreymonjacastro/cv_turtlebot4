# NAV_OFFLINE_REALISM.md — Realistic Testing Without Gazebo or the TurtleBot

## Purpose

This project needs a way to improve TurtleBot4 reactive navigation without relying on:

- the physical TurtleBot for every iteration
- the ROS/Gazebo simulator
- manual visual inspection as the primary feedback loop

The target workflow is:

```text
benchmark -> identify problem -> tweak one thing -> benchmark -> keep/reject/change direction
```

Offline validation must be realistic enough to expose likely physical failures, while remaining fast, deterministic, and safe.

## What offline tests can and cannot prove

Offline benchmarks can prove:

- LiDAR preprocessing handles invalid/noisy data.
- Safety rules trigger under known dangerous geometry.
- Navigation modules avoid obvious bad commands.
- The arbiter respects priority rules.
- Sign cooldown/debounce and QR duplicate handling work.
- A profile improved or regressed against the same scenario set.
- A controller is less oscillatory, less spin-prone, or less corner-risky than before.

Offline benchmarks cannot prove:

- the robot will physically move correctly
- `/cmd_vel` is accepted by the base
- wheel slip, latency, inertia, and sensor mounting effects are handled
- camera/YOLO/QR integration works live
- the tuned profile is competition-ready

Every report must clearly label results as offline validation unless real robot evidence is available.

## Realism strategy

Because Gazebo is not being used, realism should come from four sources:

### 1. Temporal scenario replay

Avoid single-frame tests for navigation behavior. Most real failures emerge over time:

- circling
- oscillation
- recovery loops
- repeated sign triggers
- over-correction near walls
- corner scraping
- bad turn exit alignment

Each scenario should simulate many control ticks and produce JSONL logs comparable to real `reactive_nav_debug.jsonl` logs.

### 2. Geometry stress cases

The scenario library must include difficult geometries, not only clean corridors:

- narrow corridors
- asymmetric corridors
- front-left and front-right corner blocks
- dead ends
- U-shaped traps
- close side walls
- noisy open areas
- partially blocked turns
- sign-triggered turns when the target side is unsafe

### 3. Sensor noise and invalid data

Synthetic scans must include realistic imperfections:

- `NaN`
- `inf`
- missing sectors
- sudden outliers
- range clipping
- sparse valid points
- stale timestamps
- jitter between nearby frames

The expected result is not necessarily forward progress. Sometimes the correct behavior is to stop safely and explain why.

### 4. Real robot log replay when available

If a real `reactive_nav_debug.jsonl`, `collision_events.jsonl`, or recorded `/scan` trace exists, use it to create regression cases.

The loop is:

```text
real failure log
  -> detect failure pattern
  -> create synthetic or replay scenario
  -> tune until failure is reduced offline
  -> later validate on robot
```

## Recommended offline validation levels

### Level 0 — unit tests

Tests for individual components:

- LiDAR sector extraction
- invalid range filtering
- behavior arbiter priority
- emergency stop
- turn controller phases
- QR duplicate handling
- sign debounce/cooldown

These should run with:

```bash
python3 -m pytest tests/
```

### Level 1 — synthetic scenario replay

Run all navigation modules through deterministic multi-step synthetic scenarios.

Example:

```bash
python3 scripts/replay_nav_scenarios.py \
  --nav-modules wall_follow follow_gap focm \
  --scenarios all \
  --out-dir output/sim_runs
```

This is the main fast iteration loop.

### Level 2 — real log replay

When real robot logs exist, replay/analyze them offline.

Expected scripts:

```bash
python3 scripts/analyze_robot_failure_log.py output/robot_runs/*.jsonl
python3 scripts/replay_nav_from_log.py output/robot_runs/*.jsonl --out-dir output/replay_runs
```

If these scripts are not implemented yet, Codex should add them incrementally.

### Level 3 — dry-run on robot

Only after offline profiles improve:

- run on real ROS topics
- publish zero velocity
- verify callbacks and logs
- no movement

This still does not prove movement readiness, but validates live sensor integration.

### Level 4 — low-speed physical test

Only after dry-run is clean.

## Required benchmark outputs

Every benchmark run should produce:

```text
output/<run_group>/
  *.jsonl                  per-profile/per-scenario logs
  summary.csv              machine-readable summary
  summary.md               human-readable summary
  best_profiles.md         ranked profiles and why
  regressions.md           metrics that worsened
```

Every JSONL record should include enough information to explain decisions:

- timestamp or tick
- scenario name
- profile name
- nav module
- state
- reason
- sector distances
- sector valid counts
- suggested command
- final command
- safety vetoes
- corner/yaw vetoes
- recovery state
- sign/QR state when relevant
- debug fields from the active navigation module

## Promotion rule

A tuned profile can only be promoted if it satisfies all of these:

1. Safety scenarios pass.
2. Stale/invalid LiDAR produces safe stop.
3. Corner risk is lower than baseline and ideally zero.
4. Spin ratio is lower than baseline.
5. Oscillation score is lower or acceptable.
6. It does not improve progress by removing safety.
7. It beats the current baseline on the stricter benchmark.
8. Regressions are listed clearly.

Do not overwrite existing safe configs without keeping the old files.

Use names like:

```text
wall_follow_tuned.yaml
follow_gap_tuned.yaml
wall_follow_tuned_YYYYMMDD.yaml
```

## Reporting rule

At the end of any offline improvement task, Codex must report:

- exact files changed
- exact commands run
- baseline metrics
- final metrics
- promoted profiles, if any
- rejected changes, if any
- failure modes improved
- failure modes still weak
- whether tests passed
- a clear statement that this is offline validation only
