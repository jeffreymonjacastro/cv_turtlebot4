# NAV_FAILURE_SCENARIOS.md — Offline Scenarios and Metrics for Real Robot Failure Modes

## Purpose

This document defines the failure modes the offline benchmark must expose. It exists because the robot can underperform physically even when basic synthetic scenarios pass.

The current priority is to catch:

- circling/spinning
- corner hits or scraping
- over-rotation
- oscillation in corridors
- unsafe turns near blocked sides
- recovery loops
- low progress disguised as safety

## Scenario design principles

Each scenario should be:

- deterministic by default
- multi-step over time
- fast to run
- log-producing
- tied to one or more expected failure metrics
- independent of ROS/Gazebo/physical robot

A scenario should define:

```text
name
purpose
initial synthetic scan geometry
time-varying scan changes
optional sign/QR events
expected safe behavior
failure signals
```

## Required scenarios

### `front_left_corner_blocked`

Purpose:

Detect whether the controller turns left into a close front-left obstacle.

Failure signals:

```text
front_left < threshold and angular_z > 0
corner_risk_count > 0
linear_x too high while front_left is risky
```

Expected behavior:

```text
slow down
veto or reduce positive yaw
turn away or enter recovery
```

### `front_right_corner_blocked`

Mirror of `front_left_corner_blocked`.

Failure signals:

```text
front_right < threshold and angular_z < 0
corner_risk_count > 0
```

### `corner_left_approach`

Purpose:

Simulate approaching a left corner where the front is not fully blocked but the front-left sector becomes dangerous.

Expected behavior:

```text
slow down before corner risk increases
avoid yaw into the corner
avoid scraping side wall
```

### `corner_right_approach`

Mirror of `corner_left_approach`.

### `narrow_left_turn`

Purpose:

A left sign or recovery action asks for a left turn in a narrow geometry.

Expected behavior:

```text
only turn if left/front-left clearance is safe
otherwise delay, slow, or recover
```

### `narrow_right_turn`

Mirror of `narrow_left_turn`.

### `asymmetric_corridor_left_close`

Purpose:

Detect whether wall following overreacts when the left wall is much closer than the right.

Failure signals:

```text
yaw into close left side
high oscillation
low progress
side_risk_count > 0
```

### `asymmetric_corridor_right_close`

Mirror of `asymmetric_corridor_left_close`.

### `wall_too_close_left`

Purpose:

Robot is already too close to the left wall.

Expected behavior:

```text
move slowly or stop
turn away from left wall
avoid increasing left-side risk
```

### `wall_too_close_right`

Mirror of `wall_too_close_left`.

### `u_shape_dead_end`

Purpose:

Detect recovery quality in a dead-end-like geometry.

Expected behavior:

```text
front stop triggers
recovery chooses available opening
no infinite spin
clear timeout/reason if no gap exists
```

### `spin_trap_open_space`

Purpose:

Detect circling in a scan that should permit stable forward motion.

Failure signals:

```text
spin_ratio high
mean_abs_angular_z high
commanded_distance low
angular sign changes high
```

### `noisy_corridor_with_outliers`

Purpose:

Detect overreaction to noisy LiDAR.

Scan should include:

```text
NaN
inf
occasional short outliers
sector jitter
```

Expected behavior:

```text
robust sector statistics prevent violent yaw changes
safe stop only when data is actually unusable
```

### `oscillatory_corridor`

Purpose:

Detect controllers that zig-zag between walls.

Failure signals:

```text
angular_sign_changes_per_min high
angular_smoothness_cost high
yaw_saturation_ratio high
```

## Existing safety scenarios that must remain passing

Do not weaken these while optimizing:

```text
stale_lidar
all_invalid_lidar
front_blocked
blocked_left_sign
blocked_right_sign
repeated_sign_cooldown
qr_duplicate_handling
```

## Metrics

### Safety metrics

```text
corner_risk_count
front_left_risk_count
front_right_risk_count
side_risk_count
unsafe_yaw_veto_count
min_front_distance_m
min_front_center_distance_m
min_front_left_distance_m
min_front_right_distance_m
min_left_distance_m
min_right_distance_m
stale_lidar_stop_count
all_invalid_lidar_stop_count
```

### Stability metrics

```text
spin_ratio
oscillation_score
angular_sign_changes_per_min
yaw_saturation_ratio
mean_abs_angular_z
max_abs_angular_z
angular_smoothness_cost
```

### Progress metrics

```text
commanded_distance_m
average_linear_speed_mps
low_progress_ratio
active_motion_time_s
recovery_time_ratio
```

### Behavior metrics

```text
state_transition_count_per_min
recovery_entry_count
recovery_loop_count
recovery_timeout_count
turn_count
turn_timeout_count
alignment_timeout_count
emergency_stop_count
emergency_stop_total_time_s
```

### Event metrics

```text
confirmed_sign_count
sign_retrigger_count
blocked_sign_ignored_count
stale_signal_ignored_count
qr_logged_count
qr_duplicate_ignored_count
```

## Corner risk definition

Use robot-frame convention already used by the navigation stack.

Suggested definition:

```text
front_left_risk = front_left_distance < front_corner_avoid_distance
front_right_risk = front_right_distance < front_corner_avoid_distance

unsafe_left_yaw = front_left_risk and angular_z > yaw_deadband
unsafe_right_yaw = front_right_risk and angular_z < -yaw_deadband

corner_risk_count = count(unsafe_left_yaw or unsafe_right_yaw)
```

Positive/negative yaw convention must match the implementation. If the code uses the opposite sign, adapt the metric and document it.

## Spin ratio definition

Suggested:

```text
spin_tick = abs(angular_z) > spin_yaw_threshold and abs(linear_x) < spin_linear_threshold
spin_ratio = spin_tick_count / total_tick_count
```

Exclude intentional turn states if needed:

```text
TURNING_LEFT
TURNING_RIGHT
ALIGNING_AFTER_TURN
```

Do not exclude recovery unless specifically measuring recovery spin separately.

## Oscillation definition

Suggested:

```text
angular_sign_changes = count(sign(angular_z[t]) != sign(angular_z[t-1]))
angular_sign_changes_per_min = angular_sign_changes / runtime_minutes
```

Ignore tiny values inside a deadband.

## Yaw saturation definition

Suggested:

```text
yaw_saturated = abs(angular_z) >= 0.90 * max_yaw
yaw_saturation_ratio = saturated_ticks / total_ticks
```

## Scoring guidance

Safety penalties should dominate progress rewards.

Example shape:

```text
score =
  + progress_reward
  + event_reward
  - 1000 * collision_event_count
  - 150  * corner_risk_count
  - 80   * side_risk_count
  - 60   * emergency_stop_count_in_non_safety_scenarios
  - 50   * recovery_loop_count
  - 40   * spin_ratio
  - 20   * oscillation_score
  - 20   * yaw_saturation_ratio
```

The exact weights can change, but unsafe progress must not win.

## Regression rule

If a profile improves total score but worsens any of these significantly, do not promote it automatically:

```text
corner_risk_count
side_risk_count
stale_lidar handling
all_invalid_lidar handling
spin_ratio
yaw_saturation_ratio
```

The summary must explicitly explain why it was rejected or why the regression is acceptable.
