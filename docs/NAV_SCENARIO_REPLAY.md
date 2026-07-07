# NAV_SCENARIO_REPLAY.md — Synthetic LaserScan Replay Design

## Purpose

Synthetic replay creates deterministic navigation scenarios so algorithms can be compared without the physical TurtleBot.

It should exercise:

```text
LaserScan preprocessing
sector extraction
navigation module suggestion
behavior arbitration
sign cooldown/debounce
QR behavior
stale/missing sensor safety
JSONL diagnostics
```

It must not publish `/cmd_vel`.

---

## Expected script

```text
scripts/replay_nav_scenarios.py
```

Expected CLI:

```bash
python3 scripts/replay_nav_scenarios.py   --nav-modules wall_follow follow_gap focm   --scenarios all   --out-dir output/sim_runs
```

Useful additional options:

```bash
--seed 0
--dt 0.1
--duration-s 20
--profile-name wall_follow_safe
--config ubuntu/reactive_nav/configs/wall_follow_safe.yaml
--fail-fast
```

The script should write one JSONL log per `(scenario, nav_module, profile)` pair.

Example output:

```text
output/sim_runs/open_corridor__wall_follow_safe__wall_follow.jsonl
output/sim_runs/open_corridor__follow_gap_safe__follow_gap.jsonl
output/sim_runs/open_corridor__focm_safe__focm.jsonl
```

Each file starts with a metadata record containing `scenario`, `nav_module`,
`profile_name`, `seed`, `dt_s`, `duration_s`, the effective config, and the
scenario-specific expected behavior. Step records follow.

---

## Synthetic scan object

Prefer a small dependency-free fake scan object so the script can run without ROS installed.

Minimum fields:

```python
angle_min: float
angle_max: float
angle_increment: float
range_min: float
range_max: float
ranges: list[float]
stamp: float
frame_id: str
```

If ROS messages are available, conversion to/from `sensor_msgs.msg.LaserScan` may be supported, but it should not be required.

---

## Scenario structure

Represent scenarios as small classes or functions:

```python
class Scenario:
    name: str
    duration_s: float
    dt: float

    def scan_at(self, t: float) -> FakeLaserScan:
        ...

    def signal_at(self, t: float) -> dict | None:
        ...

    def qr_at(self, t: float) -> dict | None:
        ...

    def expected(self) -> dict:
        ...
```

The runner should simulate timesteps:

```text
for t in 0..duration:
    scan = scenario.scan_at(t)
    sectors = extract_sectors(scan)
    nav_suggestion = module.compute(...)
    decision = arbiter.step(...)
    write JSONL record
```

---

## Minimum scenarios

The replay suite also includes harsh temporal failure cases from
`docs/NAV_FAILURE_SCENARIOS.md`:

```text
front_left_corner_blocked
front_right_corner_blocked
corner_left_approach
corner_right_approach
narrow_left_turn
narrow_right_turn
asymmetric_corridor_left_close
asymmetric_corridor_right_close
wall_too_close_left
wall_too_close_right
u_shape_dead_end
spin_trap_open_space
noisy_corridor_with_outliers
oscillatory_corridor
```

These scenarios are intentionally harsher than the first clean-corridor set and
are meant to expose circling, corner scrape risk, recovery loops, and noisy
LiDAR overreaction.

## `open_corridor`

Geometry:

```text
front open
left and right approximately balanced
no sign
no QR
```

Expected:

```text
no emergency stop
positive average linear speed
small average angular command
low oscillation
```

---

## `narrow_corridor`

Geometry:

```text
front open
left and right closer than open corridor
```

Expected:

```text
reduced speed if narrow-speed logic exists
no collision/emergency unless thresholds require it
low oscillation
```

---

## `left_wall_close`

Geometry:

```text
left side too close
right side open enough
front open
```

Expected:

```text
yaw away from left wall
side safety veto prevents turning further into left wall
```

---

## `right_wall_close`

Geometry:

```text
right side too close
left side open enough
front open
```

Expected:

```text
yaw away from right wall
side safety veto prevents turning further into right wall
```

---

## `front_blocked`

Geometry:

```text
front_center and front below stop/clear threshold
sides may be open
```

Expected:

```text
linear speed zero or near-zero
emergency or recovery reason logged
no unsafe forward command
```

---

## `dead_end_recovery`

Geometry:

```text
front blocked
one side gradually becomes more open
```

Expected:

```text
enter RECOVERY or equivalent
select open side
avoid blind high-speed forward command
```

---

## `left_sign_open`

Inputs:

```text
fresh repeated LEFT sign
left clearance available
```

Expected:

```text
one left maneuver triggered
cooldown prevents repeated turn
emergency can still interrupt
```

---

## `right_sign_open`

Inputs:

```text
fresh repeated RIGHT sign
right clearance available
```

Expected:

```text
one right maneuver triggered
cooldown prevents repeated turn
emergency can still interrupt
```

---

## `left_sign_blocked`

Inputs:

```text
fresh LEFT sign
left/front-left clearance poor
```

Expected:

```text
do not blindly execute unsafe turn
wait, stop, or recovery reason logged
```

---

## `right_sign_blocked`

Inputs:

```text
fresh RIGHT sign
right/front-right clearance poor
```

Expected:

```text
do not blindly execute unsafe turn
wait, stop, or recovery reason logged
```

---

## `stale_lidar`

Inputs:

```text
scan timestamp stops updating or exceeds max age
```

Expected:

```text
published command zero
stop reason clearly identifies stale LiDAR
```

---

## `all_invalid_lidar`

Inputs:

```text
ranges are NaN/inf/out of usable bounds
```

Expected:

```text
safe stop
no non-zero forward command
diagnostic explains invalid/no sector data
```

---

## `noisy_lidar_nan_inf`

Inputs:

```text
mostly valid scan
random NaN/inf/outliers
```

Expected:

```text
robust sector values remain stable enough
no crash
no unsafe command caused by one bad ray
```

---

## `qr_visible`

Inputs:

```text
QR candidate visible/decoded for multiple frames
```

Expected:

```text
QR_SCAN behavior or slowdown/stop
one persistent log event for confirmed content
duplicate sightings ignored
```

---

## `repeated_sign_cooldown`

Inputs:

```text
same sign remains visible for entire scenario
```

Expected:

```text
one maneuver
no repeated maneuver during cooldown
```

---

## JSONL record shape

Each replay log should include enough fields to match real run debugging.

Minimum recommended fields:

```json
{
  "timestamp": 0.0,
  "scenario": "open_corridor",
  "profile_name": "offline_default",
  "nav": {
    "module": "wall_follow",
    "suggested_linear_x": 0.10,
    "suggested_angular_z": 0.02,
    "debug": {}
  },
  "state": "CORRIDOR_FOLLOW",
  "reason": "FRONT_CLEAR",
  "lidar": {
    "front_center_m": 1.20,
    "front_m": 1.15,
    "left_m": 0.55,
    "right_m": 0.58,
    "num_valid": 360,
    "age_s": 0.0
  },
  "signal": {
    "direction": "none",
    "fresh": false
  },
  "qr": {
    "visible": false,
    "logged": false
  },
  "command": {
    "requested_linear_x": 0.10,
    "requested_angular_z": 0.02,
    "published_linear_x": 0.10,
    "published_angular_z": 0.02
  },
  "emergency": {
    "active": false,
    "reason": "NONE"
  },
  "dry_run": true,
  "enable_motion": false
}
```

Offline replay does not physically publish. The `published_*` fields mean “what would have been published after arbitration.”

The implemented replay also includes:

```text
command.motion_published_to_robot=false
command.publication_mode=offline_would_publish_after_arbitration
mode_flags.dry_run=true
mode_flags.enable_motion=false
```

---

## Determinism

By default, scenarios must be deterministic.

For noisy scenarios:

```text
accept --seed
write seed into metadata
use local random generator, not global random state
```

---

## Acceptance criteria

Synthetic replay is acceptable when:

```text
all required scenarios are implemented
all nav modules can run against all scenarios
logs are valid JSONL
compare script can read the logs
offline scripts do not require a robot
offline scripts do not publish /cmd_vel
failure reasons are visible in logs
```
