# NAV_RECOVERY_REPLAY_AND_ABLATION.md — Replay and Ablation for Turn/Recovery Failures

## Purpose

Use robot logs to determine whether a proposed code/config change would have made better decisions during the exact intervals where the robot got stuck.

This is offline validation. It does not prove physical success, but it is much faster than repeated robot attempts.

---

## Required scripts

Codex should add or extend these scripts if missing:

```text
scripts/analyze_robot_failure_log.py
scripts/extract_turn_recovery_intervals.py
scripts/replay_turn_recovery_intervals.py
scripts/run_turn_recovery_ablation.py
```

---

## Failure interval extraction

Detect intervals matching:

```text
TURNING_LEFT or TURNING_RIGHT or ALIGNING_AFTER_TURN
-> FRONT_BLOCKED_SELECT_FREE_GAP or RECOVERY
-> remains stuck, times out, loops, or fails to return to NAVIGATE
```

Also detect:

```text
RECOVERY duration > threshold
RECOVERY <-> NAVIGATE loop
FRONT_BLOCKED_SELECT_FREE_GAP repeated entries
high yaw + low linear outside intentional turn
selected gap angle repeatedly changes sign
selected gap points toward close side/corner
```

Output:

```text
output/turn_recovery_analysis/failure_intervals.jsonl
output/turn_recovery_analysis/failure_summary.md
output/turn_recovery_analysis/suggested_changes.md
```

Each interval record should include:

```text
run_name
start_time
end_time
duration_s
initial_state
terminal_state
state_sequence
profile_name
nav_module
min_front
min_front_left
min_front_right
min_left
min_right
max_abs_yaw
avg_linear
recovery_entry_count
front_blocked_select_count
suspected_cause
```

---

## Replay modes

### Sector-level replay

Use when logs contain only sector distances.

```text
logged sector distances
-> build navigation observation
-> run new controller/arbiter decision logic
-> compare old vs new requested command/state transition
```

Label output clearly:

```text
replay_type=sector_level
```

### Scan-level replay

Use when `/scan` bags or converted scan JSONL exist.

```text
raw LaserScan
-> scan_to_points / lidar_sectors with angle_offset
-> nav module
-> arbiter
```

Label output:

```text
replay_type=scan_level
```

Scan-level replay is preferred because it validates `angle_offset` and sector extraction.

---

## Ablation protocol

Run one change at a time:

```text
baseline_current
angle_offset_only
turn_recovery_delay
turn_front_block_tolerance
turn_min_commitment_time
recovery_exit_relaxed
gap_scoring_adjusted
angular_smoothing_adjusted
combined_safe_candidate
```

For each ablation, report:

```text
turn_success_proxy
recovery_entries_during_turn
max_recovery_duration_s
recovery_timeout_count
front_blocked_select_count
corner_risk_count
side_risk_count
spin_ratio
oscillation_score
safety_regression_count
```

Do not promote a combined candidate unless it beats baseline and individual ablations without safety regression.

---

## Example commands

```bash
python3 scripts/extract_turn_recovery_intervals.py \
  output/robot_runs/*/reactive_nav_debug.jsonl \
  --out-dir output/turn_recovery_analysis

python3 scripts/replay_turn_recovery_intervals.py \
  --intervals output/turn_recovery_analysis/failure_intervals.jsonl \
  --profiles wall_follow_tuned follow_gap_safe \
  --out-dir output/turn_recovery_replay

python3 scripts/run_turn_recovery_ablation.py \
  --intervals output/turn_recovery_analysis/failure_intervals.jsonl \
  --out-dir output/turn_recovery_ablation
```

---

## Promotion rule

A change may be recommended for robot dry-run only if:

```text
unit tests pass
synthetic benchmark does not regress safety
turn/recovery replay improves stuck intervals
corner/side risk does not increase
logs show why recovery enters and exits
```

A change may be recommended for physical movement only after robot dry-run shows fresh callbacks, sane state transitions, and no obvious recovery loop while stationary.
