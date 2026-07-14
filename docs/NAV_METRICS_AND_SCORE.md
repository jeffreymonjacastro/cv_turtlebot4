# NAV_METRICS_AND_SCORE.md — Navigation Metrics and Profile Comparison

## Purpose

This document defines the metrics used to compare navigation algorithms and profiles.

The metrics are meant to answer:

```text
Is it safe?
Does it make progress?
Does it oscillate?
Does it recover?
Does it obey signs/QR behavior?
Is this profile ready for dry-run or physical testing?
```

The score is only a ranking heuristic. Raw logs and failure cases remain the source of truth.

---

## Input

The comparison script should accept one or more JSONL logs:

```bash
python3 scripts/compare_nav_profiles.py output/sim_runs/*.jsonl
```

Each input log should contain records from one run.

The script should infer or read:

```text
scenario
profile_name
nav_module
timestamps
state
reason
lidar fields
command fields
emergency fields
turn fields, if present
QR/sign fields, if present
```

---

## Required output

Print a table with one row per run:

```text
scenario
profile
module
runtime_s
score
emergency_count
recovery_ratio
avg_v
mean_abs_w
oscillation
min_front
turns
timeouts
stale_stops
```

Optionally also write:

```text
output/nav_comparison_summary.csv
output/nav_comparison_summary.json
```

The current comparison script writes both files by default. Use
`--no-write-summary` for read-only console inspection, or `--json` to print the
full metric payload.

For iteration runs, write explicit summary targets:

```bash
python3 scripts/compare_nav_profiles.py output/iter_final/*.jsonl \
  --summary-csv output/iter_final/summary.csv \
  --summary-json output/iter_final/summary.json \
  --summary-md output/iter_final/summary.md \
  --baseline output/iter_harsh_baseline/summary.csv
```

---

## Core metrics

## Runtime

```text
total_runtime_s = last_timestamp - first_timestamp
```

If timestamps are missing, use record index and configured `dt`.

---

## Time per state

For each state:

```text
time_in_state[state] = sum(dt for records in state)
```

Important states:

```text
SENSOR_CHECK
CORRIDOR_FOLLOW
NAVIGATE
RECOVERY
EMERGENCY_STOP
TURNING_LEFT
TURNING_RIGHT
ALIGNING_AFTER_TURN
QR_SCAN
MANUAL_STOP
IDLE
```

If the current code uses different state names, preserve them but map common aliases when possible.

---

## Emergency metrics

```text
emergency_stop_count
emergency_stop_total_time_s
emergency_trigger_reasons
```

Count transitions into emergency, not every record.

---

## Recovery metrics

```text
recovery_time_ratio = time_in_RECOVERY / total_runtime_s
```

High recovery ratio in ordinary corridor scenarios is bad.

---

## Speed/progress metrics

Use command fields after arbitration:

```text
average_published_linear_speed_mps
max_published_linear_speed_mps
commanded_distance_estimate_m = sum(max(0, linear_x) * dt)
active_motion_time_s = time where abs(linear_x) > epsilon or abs(angular_z) > epsilon
```

If this is an offline replay, this is only commanded progress, not real displacement.

---

## Smoothness metrics

```text
mean_abs_angular_speed_radps = mean(abs(angular_z))
angular_sign_changes_per_min = sign changes in angular_z per minute
linear_speed_variance
oscillation_score
```

Suggested oscillation score:

```text
oscillation_score =
  angular_sign_changes_per_min
  + 2.0 * mean_abs_angular_speed_radps
  + 5.0 * linear_speed_variance
```

This is heuristic. Keep raw components visible.

---

## Safety distance metrics

Use sector values from logs:

```text
minimum_front_distance_m
minimum_front_center_distance_m
minimum_left_distance_m
minimum_right_distance_m
minimum_side_distance_m = min(minimum_left_distance_m, minimum_right_distance_m)
```

If sector values are missing, report `NA`.

Harsh failure scenarios also track:

```text
corner_risk_count
front_left_risk_count
front_right_risk_count
side_risk_count
unsafe_yaw_veto_count
spin_ratio
yaw_saturation_ratio
angular_smoothness_cost
low_progress_ratio
recovery_entry_count
recovery_loop_count
recovery_timeout_count
state_transition_count_per_min
minimum_front_left_distance_m
minimum_front_right_distance_m
```

---

## Safety veto metrics

Count cases where the arbiter changed or suppressed a command:

```text
unsafe_command_veto_count
forward_command_veto_count
yaw_veto_count
stale_lidar_stop_count
invalid_lidar_stop_count
```

If logs do not expose these explicitly, infer where possible from:

```text
nav.suggested_* vs command.published_*
state/reason strings
emergency.reason
```

Prefer explicit fields over inference.

---

## Turn metrics

```text
turn_count
left_turn_count
right_turn_count
average_turn_duration_s
turn_timeout_count
alignment_timeout_count
turn_completion_reasons
```

Count a turn when the state transitions into `TURNING_LEFT` or `TURNING_RIGHT`.

A repeated sign should not cause repeated turns during cooldown.

---

## Sign metrics

```text
sign_candidate_count
confirmed_sign_count
stale_signal_ignored_count
sign_retrigger_count
cooldown_suppression_count
blocked_turn_suppression_count
```

Use these to debug YOLO integration without the robot.

---

## QR metrics

```text
qr_visible_time_s
qr_logged_count
qr_duplicate_ignored_count
qr_scan_time_s
```

QR metrics matter because QR is project evidence, not just a side feature.

---

## Scenario score

Use a simple score for ranking, but do not overfit blindly.

Suggested first version:

```text
score =
  + 100 * completed_scenario
  + 20  * commanded_distance_estimate_m
  + 10  * confirmed_sign_count
  + 10  * qr_logged_count
  - 100 * collision_event_count
  - 50  * unsafe_forward_while_blocked_count
  - 30  * emergency_stop_count
  - 25  * stale_lidar_motion_violation_count
  - 20  * recovery_time_ratio
  - 10  * oscillation_score
  - 10  * turn_timeout_count
  - 10  * alignment_timeout_count
```

`completed_scenario` should be scenario-specific. Examples:

```text
open_corridor:
  enough commanded distance, no emergency, low oscillation

front_blocked:
  safe stop/recovery, no unsafe forward motion

left_sign_open:
  exactly one left turn, no repeated cooldown violation

stale_lidar:
  zero motion after stale condition
```

For safety scenarios, “completion” may mean stopping correctly, not moving far.

---

## Readiness classification

The comparison script can classify each run:

```text
PASS
WARN
FAIL
```

Suggested rules:

## FAIL

```text
unsafe forward while front blocked
non-zero motion with stale/invalid LiDAR
missing required log fields
crash/exception
repeated sign turn during cooldown
```

## WARN

```text
high oscillation
high recovery ratio in open corridor
turn timeout
alignment timeout
very low commanded progress in ordinary corridor
```

## PASS

```text
scenario-specific acceptance criteria met
no safety violations
logs complete enough to debug
```

Scenario status is intentionally conservative. Synthetic replay can mark a run
`PASS` only for offline validation; it is not a physical robot guarantee.

---

## Output example

```text
scenario          module       score  status  emerg  recov%  avg_v  osc   min_front  turns  notes
open_corridor     wall_follow   122.5  PASS    0      0.00    0.10   1.2   1.10       0      -
open_corridor     follow_gap     80.3  WARN    0      0.15    0.07   4.8   1.05       0      oscillation high
front_blocked     wall_follow    70.0  PASS    1      0.60    0.00   0.5   0.22       0      safe stop
stale_lidar       focm         -100.0  FAIL    0      0.00    0.04   0.2   NA         0      moved with stale lidar
```

---

## Interpretation rules

Do not select a module only by highest total score.

Prefer a profile that:

```text
never violates safety
has stable behavior across scenarios
has readable failure reasons
uses recovery only when needed
does not oscillate excessively
handles stale/missing sensors safely
```

A slower but predictable profile is better than a faster profile that occasionally violates safety.
