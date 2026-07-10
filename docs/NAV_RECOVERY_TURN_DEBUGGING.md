# NAV_RECOVERY_TURN_DEBUGGING.md — Turn Completion and Gap Recovery Debugging

## Purpose

This document defines the next focused iteration for the TurtleBot4 reactive navigation stack.

The current problem is not generic navigation quality. The specific failure mode is:

```text
robot starts or approaches a turn
-> front appears blocked or partially blocked
-> arbiter enters FRONT_BLOCKED_SELECT_FREE_GAP or RECOVERY
-> robot stays there too long or never completes the turn
```

The goal is to make turns complete reliably by understanding whether the issue is:

1. poor LiDAR alignment / angle offset,
2. overly aggressive front-blocked recovery during active turns,
3. bad free-gap selection,
4. recovery exit conditions that are too strict or missing,
5. yaw commands blocked by safety vetoes,
6. physical environment geometry not represented in synthetic tests.

Do not start with maze simulation. Start with real logs and recovery-specific diagnostics.

---

## Robot capture commands

Use the capture workflow in `docs/NAV_REAL_DATA_CAPTURE_FOR_TURNS.md` for the actual robot session. The minimum sequence is:

```bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu/reactive_nav_test
```

Then run the `wall_follow_tuned` capture command that matches the scenario:

```text
angle_offset dry-run check
isolated left turn
isolated right turn
front-blocked recovery
```

Use the `reactive_nav_debug.jsonl` and optional rosbag from each run as the evidence source for interval extraction and replay.

---

## Main hypothesis

The highest-priority hypothesis is:

```text
During intentional turns or post-turn alignment, normal front-blocked recovery is taking over too aggressively.
```

During a legitimate left/right turn in a corridor, the front sector can temporarily look blocked because the robot is rotating near a corner. This should not always trigger full free-gap recovery. It should only interrupt the turn when emergency-level collision thresholds are crossed.

---

## Required diagnostic fields

Every tick in `FRONT_BLOCKED_SELECT_FREE_GAP`, `RECOVERY`, `TURNING_LEFT`, `TURNING_RIGHT`, and `ALIGNING_AFTER_TURN` should log enough information to answer:

```text
Why did we enter recovery?
Which gap was selected?
Why was this gap selected?
What command did recovery request?
Did the arbiter veto or modify it?
Why did recovery not exit?
```

Add or verify these fields in the persistent JSONL log:

```text
state
reason
prev_state
state_duration_s
profile_name
nav.module
lidar.angle_offset_deg
lidar.front_center
lidar.front
lidar.front_left
lidar.front_right
lidar.left
lidar.right
lidar.min_clearance_m
turn.turn_active
turn.turn_direction
turn.turn_phase
turn.elapsed_s
turn.heading_error_deg or turn.progress_estimate_deg if available
turn.completed_reason
turn.abort_reason
recovery.entry_reason
recovery.elapsed_s
recovery.gap_count
recovery.best_gap_center_deg
recovery.best_gap_width_deg
recovery.best_gap_score
recovery.best_gap_min_range_m
recovery.selected_gap_is_left
recovery.selected_gap_is_right
recovery.front_clear
recovery.exit_candidate
recovery.exit_reason
recovery.block_reason
recovery.timeout
nav.suggested_linear_x
nav.suggested_angular_z
command.requested_linear_x
command.requested_angular_z
command.published_linear_x
command.published_angular_z
arbiter.veto_reason
arbiter.corner_veto_active
arbiter.side_veto_active
emergency.emergency_active
emergency.emergency_trigger_reason
```

If some fields are unavailable, log `null` and document why.

---

## Turn-specific recovery rule to evaluate

Candidate rule:

```text
If state is TURNING_LEFT, TURNING_RIGHT, or ALIGNING_AFTER_TURN:
    do not enter FRONT_BLOCKED_SELECT_FREE_GAP merely because front < normal front_clear threshold
    allow turn controller to continue if emergency threshold is not crossed
    apply corner/side safety vetoes as usual
    interrupt only on emergency stop, severe side scrape risk, timeout, or no progress
```

This must be tested as an ablation, not blindly accepted.

Suggested ablation names:

```text
baseline_current
turn_recovery_delay
turn_front_block_tolerance
turn_min_commitment_time
recovery_exit_relaxed
gap_scoring_adjusted
combined_turn_recovery
```

---

## Recovery exit conditions

Recovery should not become a trap state.

A recovery tick should report whether each exit condition is met:

```text
front clear enough
front_left/front_right safe enough
selected gap near forward enough
state duration above minimum recovery duration
state duration below timeout
command not yaw-saturated for too long
not repeatedly toggling between recovery and navigate
```

Possible exit condition:

```text
exit recovery if:
    front > front_recovery_clear_distance
    and front_left > front_corner_clear_distance
    and front_right > front_corner_clear_distance
    and at least min_recovery_s elapsed
```

But if the robot is in an intentional turn, recovery may need to return to `ALIGNING_AFTER_TURN` rather than directly to `NAVIGATE`.

---

## What counts as improvement

A change is useful only if it improves the failure mode without weakening safety.

Track:

```text
turn_success_rate
turn_completion_time_s
recovery_entries_during_turn
front_blocked_recovery_entries_during_turn
max_recovery_duration_s
recovery_timeout_count
state_loop_count
corner_risk_count
side_risk_count
emergency_stop_count
spin_ratio
oscillation_score
```

Promote a change only if:

```text
safety scenarios still pass
corner/side risk do not increase
turn failure intervals improve
recovery duration decreases or exits become correct
no new infinite loops appear
```

---

## What not to do

Do not:

```text
increase speed to force progress
remove emergency stop thresholds
disable recovery globally
disable corner/side vetoes
claim the issue is fixed without robot validation
optimize only synthetic scores
make maze simulation the first step
```
