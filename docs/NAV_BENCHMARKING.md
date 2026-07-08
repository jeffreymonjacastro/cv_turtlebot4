# NAV_BENCHMARKING.md — Systematic Navigation Evaluation

## Purpose

This document defines how to test and compare TurtleBot4 reactive navigation algorithms without relying on constant physical robot runs.

The goal is faster iteration:

```text
change algorithm
-> run unit tests
-> run synthetic LaserScan scenarios
-> compare logs/metrics
-> replay real scans or simulate
-> only then test on the physical robot
```

Physical robot testing is still necessary, but it should not be the first debugging loop.

---

## Current architecture assumed by the benchmark

The benchmark assumes the navigation stack is organized as:

```text
LaserScan or synthetic scan
  -> lidar_sectors.py
  -> selected navigation module
  -> behavior_arbiter.py
  -> command decision
  -> JSONL log
```

The selected navigation module may be:

```text
wall_follow
follow_gap
focm
```

or any future module registered in the same factory.

The navigation module must only produce a suggested command and debug information. Safety, sign handling, QR behavior, and command publication remain outside the module.

---

## Test levels

## Level 0 — unit tests

Fastest checks. No ROS. No robot. No simulator.

Use for:

```text
sector extraction
invalid LiDAR filtering
emergency stop conditions
sign debounce/cooldown
turn controller phases
QR duplicate handling
navigation module command signs
```

Expected command:

```bash
python3 -m pytest tests/
```

Minimum test files:

```text
tests/test_lidar_sectors.py
tests/test_behavior_arbiter.py
tests/test_turn_controller.py
tests/test_qr_logger.py
tests/test_nav_modules_synthetic.py
```

Acceptance:

```text
all tests pass
no test publishes /cmd_vel
tests are deterministic
```

---

## Level 1 — synthetic LaserScan scenario replay

This is the first main benchmark.

It creates fake `LaserScan`-like inputs and runs the same sector extraction, navigation module, and arbiter logic used by the robot node.

Expected command:

```bash
python3 scripts/replay_nav_scenarios.py   --nav-modules wall_follow follow_gap focm   --scenarios all   --out-dir output/sim_runs
```

Then compare:

```bash
python3 scripts/compare_nav_profiles.py output/sim_runs/*.jsonl
```

The replay writes one JSONL file per `(scenario, profile, module)` combination.
Records are offline diagnostics only: `command.published_*` means the command
that would have been sent after arbitration, not a real `/cmd_vel` publication.
Every replay record must keep:

```text
dry_run=true
enable_motion=false
command.motion_published_to_robot=false
```

Use this for:

```text
fast algorithm comparison
regression tests
tuning parameters
detecting sign mistakes
detecting oscillation
checking stale/missing sensor safety
checking repeated sign cooldown
```

Do not use this to claim real-world performance. It is offline validation.

Comparison summaries are written to:

```text
output/nav_comparison_summary.csv
output/nav_comparison_summary.json
```

Use the printed `status` column as a triage signal. Inspect the raw JSONL before
trusting a score or changing robot-side behavior.

---

## Level 2 — recorded real `/scan` replay

This uses real LiDAR data without moving the robot.

Record on the TurtleBot:

```bash
ros2 bag record /scan -o bags/corridor_open
ros2 bag record /scan -o bags/narrow_corridor
ros2 bag record /scan -o bags/front_blocked
ros2 bag record /scan -o bags/turn_area
```

Then either replay directly if ROS bag dependencies are available, or convert to a simple JSONL scan trace and run:

```bash
python3 scripts/replay_scan_log.py   --scan-log output/scan_logs/corridor_open.jsonl   --nav-modules wall_follow follow_gap focm   --out-dir output/real_scan_replay
```

Use this for:

```text
real LiDAR noise
real NaN/inf patterns
real corridor geometry
front/side sector instability
debugging cases where synthetic scans were too clean
```

Acceptance:

```text
same recorded scan sequence can be replayed against multiple modules
outputs comparable JSONL logs
no motion is published
```

---

## Level 3 — Gazebo/TurtleBot simulation

Use simulation when the controller needs closed-loop movement feedback.

Use this for:

```text
does the robot actually move through a corridor?
does recovery escape a blocked front?
does a turn overshoot or undershoot?
does the controller oscillate in closed loop?
does the profile scrape corners in a maze-like world?
```

Suggested structure:

```text
same reactive_navigator.py
same nav_module parameter
same safety hierarchy
simulated /scan
simulated /cmd_vel
sim logs under output/sim_runs or output/gazebo_runs
```

Important: keep simulation isolated from the physical TurtleBot using ROS domain isolation or local-only ROS settings.

---

## Level 4 — physical robot validation

Only run on the robot after the profile passes offline checks.

Recommended flow:

```text
1. dry-run on robot
2. verify fresh callbacks and logs
3. low-speed open-space motion test
4. short corridor test
5. sign/turn test
6. integrated circuit attempt
```

Minimum evidence to save:

```text
/home/ubuntu/output/reactive_nav_debug.jsonl
/home/ubuntu/output/collision_events.jsonl
/home/ubuntu/output/qr_log.jsonl
camera/YOLO evidence if relevant
profile/config used
```

Copy back:

```bash
scp turtlebot4:/home/ubuntu/output/reactive_nav_debug.jsonl output/robot_runs/<name>.jsonl
python3 scripts/compare_nav_profiles.py output/robot_runs/<name>.jsonl
```

---

## Readiness rule for physical testing

Do not physically test a profile unless:

```text
unit tests pass
normal synthetic scenarios do not trigger unsafe motion
blocked/stale/invalid scenarios stop safely
repeated signs do not retrigger during cooldown
recovery does not dominate ordinary corridor scenarios
oscillation score is not extreme
logs explain every stop, recovery, and turn
```

A profile may still fail on the robot. The point is to avoid wasting robot time on failures that are visible offline.

---

## Recommended daily iteration loop

```bash
git status --short --branch

python3 -m pytest tests/

python3 scripts/replay_nav_scenarios.py   --nav-modules wall_follow follow_gap focm   --scenarios all   --out-dir output/sim_runs

python3 scripts/compare_nav_profiles.py output/sim_runs/*.jsonl
```

Then inspect the worst scenario logs manually before changing code again.

## Deterministic tuning

Use bounded random search only after the harsh benchmark exposes failure modes:

```bash
python3 scripts/tune_nav_profiles.py \
  --nav-modules wall_follow follow_gap \
  --scenarios all \
  --trials 100 \
  --seed 7 \
  --out-dir output/tuning_runs
```

The tuner rejects unsafe candidates and only exports new configs such as
`ubuntu/reactive_nav/configs/wall_follow_tuned.yaml` when a safe candidate beats
that module's harsh-baseline score.

## Real/debug log analysis

When robot/debug logs exist, analyze and optionally replay them offline:

```bash
python3 scripts/analyze_robot_failure_log.py output/reactive_nav_debug*.jsonl

python3 scripts/extract_real_log_regressions.py output/reactive_nav_debug*.jsonl \
  --intervals output/real_log_analysis/failure_intervals.jsonl \
  --out-dir output/real_log_regressions

python3 scripts/replay_real_log_nav.py output/reactive_nav_debug*.jsonl \
  --profiles wall_follow_safe wall_follow_tuned follow_gap_safe \
  --out-dir output/real_log_replay

python3 scripts/run_nav_ablation.py --out-dir output/ablation_runs/real_log_iter
```

The repeatable loop is:

```text
real logs
-> failure_intervals.jsonl
-> representative regression snippets
-> sector-level replay
-> ablation report
-> tuned config decision
```

Sector-level log replay and ablation are still offline validation only.

---

## What to report after each navigation change

Every navigation-related Codex report should include:

```text
files changed
commands run
test result summary
scenario result summary
best/worst profiles
remaining failure cases
whether robot dry-run is recommended
whether physical movement is recommended
```

Avoid vague claims like “improved navigation.” Use metrics.
