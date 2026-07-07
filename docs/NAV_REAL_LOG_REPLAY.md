# NAV_REAL_LOG_REPLAY.md — Using Robot Logs to Improve Offline Benchmarks

## Purpose

Real robot logs are the best source of realism when Gazebo and the physical robot are not part of the iteration loop.

Use real logs to answer:

- Did the robot spin?
- Did it turn into a close wall/corner?
- Was the LiDAR stale or noisy?
- Did emergency stop trigger too late or too often?
- Did recovery loop indefinitely?
- Did sign handling retrigger?

Then convert those patterns into offline regression scenarios.

## Expected input logs

Common files:

```text
reactive_nav_debug.jsonl
collision_events.jsonl
qr_log.jsonl
```

Useful fields, if present:

```text
state
reason
profile_name
nav.module
lidar.front
lidar.front_center
lidar.front_left
lidar.front_right
lidar.left
lidar.right
nav.suggested_linear_x
nav.suggested_angular_z
command.requested_linear_x
command.requested_angular_z
command.published_linear_x
command.published_angular_z
emergency.emergency_active
emergency.emergency_trigger_reason
turn.turn_phase
turn.turn_direction
signal
qr
```

Field names may differ. Analysis scripts should be tolerant and report missing fields.

## Required analyzer

Add or maintain:

```bash
python3 scripts/analyze_robot_failure_log.py output/robot_runs/*.jsonl
```

The script should produce:

```text
output/robot_failure_analysis/
  failure_summary.md
  failure_intervals.csv
  suggested_scenarios.md
```

## Failure intervals

Detect intervals for:

### Circling / spinning

```text
high abs(angular_z)
low abs(linear_x)
not in intentional TURNING_* state
lasts more than N seconds
```

### Corner risk

```text
front_left close and yaw left
front_right close and yaw right
```

### Side scrape risk

```text
left close and yaw left
right close and yaw right
```

### Recovery loop

```text
state repeatedly enters RECOVERY
or remains in RECOVERY too long
or alternates RECOVERY <-> NAVIGATE frequently
```

### Emergency bursts

```text
many emergency stops in a short interval
or emergency never clears
```

### Oscillation

```text
angular_z sign flips frequently
```

## Suggested scenarios from logs

The analyzer should recommend synthetic scenarios based on observed patterns.

Examples:

```text
Pattern: front_left close + positive yaw before collision
Suggested scenario: front_left_corner_blocked
```

```text
Pattern: high yaw + low linear for 6 seconds in open-looking scan
Suggested scenario: spin_trap_open_space
```

```text
Pattern: recovery repeatedly enters/exits with front blocked
Suggested scenario: u_shape_dead_end
```

## Replay from logs

If the debug log includes enough sector distances, the project can replay from sector-level observations even without full `/scan` data.

Useful script:

```bash
python3 scripts/replay_nav_from_log.py \
  output/robot_runs/reactive_nav_debug.jsonl \
  --nav-modules wall_follow follow_gap focm \
  --out-dir output/log_replay_runs
```

This should:

- parse sector distances from each tick
- build navigation observations
- run selected modules on the same observations
- compare suggested/final commands with the original run
- produce JSONL and summary metrics

If full LaserScan is unavailable, label it clearly as sector-level replay.

## Regression scenario creation

When a real failure pattern is discovered, add a deterministic scenario that captures the geometry.

Minimum process:

```text
1. identify failure interval in real log
2. summarize relevant sector values
3. create synthetic scenario with similar sector evolution
4. add expected failure metric
5. run benchmark before/after fix
```

## Reporting rule

Codex must distinguish:

```text
real robot observation
offline reproduction
offline improvement
physical validation pending
```

Do not claim a real issue is fixed until it has been validated physically.
