# Navigation Real-Log Regression

## Purpose

Real robot failures are the highest-value offline test data available when we are not using Gazebo or the TurtleBot.

The goal is to transform physical-run failures into repeatable offline tests:

```text
physical failure
  -> JSONL interval
  -> replay/regression case
  -> controller response comparison
  -> accepted or rejected change
```

This makes the project improve through evidence rather than trial-and-error.

## Inputs

Preferred input files:

```text
output/reactive_nav_debug.jsonl
output/reactive_nav_debug_*.jsonl
output/collision_events.jsonl
output/collision_events_*.jsonl
output/collision_frames/
```

The debug log should contain, when available:

```text
timestamp / time
state / reason
sector distances
valid sector counts
nav suggested command
arbiter command
published command
emergency fields
turn fields
signal fields
QR fields
```

If full raw `/scan` is not present, use sector-level replay. Sector-level replay is still useful because many failures are caused by decisions from already-computed sector distances.

## Two replay modes

### 1. Sector replay

Use existing fields like:

```text
front_center
front
front_left
front_right
left
right
rear_left
rear_right
```

Then rebuild a `NavigationObservation` or equivalent internal object and run:

```text
observation
  -> selected nav module
  -> behavior arbiter
  -> simulated command
```

This is enough to test:

```text
corner veto
side risk veto
anti-spin behavior
recovery entry
recovery exit
sign cooldown behavior
QR behavior
```

### 2. Scan replay

If raw LaserScan samples were logged or can be reconstructed, replay:

```text
LaserScan-like object
  -> lidar_sectors
  -> nav module
  -> behavior arbiter
```

This is better for testing sector extraction and robustness to invalid/noisy ranges.

## Failure interval detection

Detect and save intervals for:

### Corner risk

```text
front_left close and command turns left
front_right close and command turns right
```

Example rule:

```text
front_left < front_corner_avoid_distance and angular_z > yaw_epsilon
front_right < front_corner_avoid_distance and angular_z < -yaw_epsilon
```

### Side scrape risk

```text
left < side_avoid_distance and yaw turns toward left
right < side_avoid_distance and yaw turns toward right
```

### Spin interval

```text
abs(angular_z) >= spin_yaw_threshold
and abs(linear_x) <= spin_linear_threshold
for at least spin_window_s
outside intentional TURNING_LEFT / TURNING_RIGHT states
```

### Oscillation interval

```text
angular_z sign changes many times within a short window
```

### Yaw saturation

```text
abs(angular_z) >= 0.90 * max_yaw
for too much of the interval
```

### Recovery loop

```text
state enters RECOVERY repeatedly
or stays in RECOVERY without returning to stable NAVIGATE / CORRIDOR_FOLLOW
```

### Emergency burst

```text
EMERGENCY_STOP triggers repeatedly within a short window
```

## Output artifacts

The analyzer/replay scripts should write:

```text
output/real_log_analysis/<run_id>/failure_intervals.jsonl
output/real_log_analysis/<run_id>/summary.md
output/real_log_analysis/<run_id>/metrics.csv
output/real_log_replay/<run_id>/<profile>.jsonl
output/real_log_replay/<run_id>/comparison.md
```

Each failure interval record should include:

```json
{
  "run_id": "...",
  "failure_type": "corner_risk",
  "start_time": 0.0,
  "end_time": 1.5,
  "duration_s": 1.5,
  "state_counts": {},
  "min_front_left_m": 0.22,
  "min_front_right_m": 0.88,
  "min_left_m": 0.40,
  "min_right_m": 0.55,
  "mean_linear_x": 0.03,
  "mean_angular_z": 0.42,
  "representative_record_index": 1234,
  "notes": "turning into close front-left corner"
}
```

## Regression case extraction

For each common failure class, extract a small number of representative cases:

```text
worst corner-risk interval
longest spin interval
highest oscillation interval
closest side-scrape interval
longest recovery loop
most frequent emergency-stop burst
```

Then create regression scenarios from those intervals.

Scenario names should encode origin:

```text
real_corner_risk_runA_001
real_spin_runA_002
real_oscillation_runA_003
real_side_scrape_runA_004
```

## Replay comparison

For each candidate profile, compare:

```text
old command in log
new simulated command from current controller
```

Key deltas:

```text
corner risk: old count -> new count
side risk: old count -> new count
spin ratio: old -> new
oscillation score: old -> new
yaw saturation ratio: old -> new
recovery loop count: old -> new
mean linear speed: old -> new
```

A good result is not merely higher speed. A good result reduces unsafe or unstable decisions while keeping reasonable progress.

## Acceptance

A real-log replay improvement is acceptable if:

```text
corner risk decreases or remains zero
side risk decreases or remains zero
spin intervals decrease
oscillation does not increase materially
yaw saturation does not increase materially
safety cases still pass in synthetic benchmark
```

Reject changes that only improve synthetic score while making real-log replay worse.
