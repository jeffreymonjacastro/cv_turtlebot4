# docs/NAVIGATION_ARCHITECTURE.md

## Objective

Implement a lightweight autonomous navigation system for TurtleBot 4 Lite using:

```text
LiDAR as the primary safety/navigation sensor
YOLO as a traffic-sign event detector
QR decoding as checkpoint/evidence logic
A behavior arbiter as the only module allowed to command motion
```

The system must be robust enough for an unknown circuit and simple enough to debug under project pressure.

## Why not use a classic maze solver?

A classic maze solver assumes a clean maze abstraction. This project has:

- traffic signs that modify route choice
- QR checkpoint objectives
- unknown obstacles/boxes
- imperfect camera detections
- penalties for collisions/failure
- limited onboard compute

Therefore the default behavior should be reactive navigation, not full graph solving.

Use:

```text
corridor/wall following
+ LiDAR free-space recovery
+ sign-triggered maneuvers
```

## High-level data flow

```text
Camera stream
  -> YOLO detector
  -> latest sign state
  -> behavior arbiter

Camera stream
  -> QR detector
  -> QR event / QR log
  -> behavior arbiter

LiDAR LaserScan
  -> sector extraction
  -> safety distances
  -> wall/corridor controller
  -> behavior arbiter

Behavior arbiter
  -> Twist command
  -> /cmd_vel or existing command interface

Diagnostics
  -> console + UDP LOG/LIDAR packets
  -> laptop receiver
```

## Control loop rates

Do not bind navigation to camera FPS.

Recommended rates:

```text
LiDAR safety/controller: 10-20 Hz
Behavior arbiter:        10-20 Hz
YOLO detector:            5-10 Hz
QR detector:              5-15 Hz
Diagnostics:              2-10 Hz
```

If YOLO stalls, LiDAR safety and navigation must continue.

## State machine

Minimum states:

```text
INIT
SENSOR_CHECK
NAVIGATE
SIGN_CANDIDATE
TURNING_LEFT
TURNING_RIGHT
ALIGNING_AFTER_TURN
QR_SCAN
RECOVERY
EMERGENCY_STOP
MANUAL_STOP
```

### State meanings

#### INIT

Load parameters. Do not move.

#### SENSOR_CHECK

Verify required interfaces:

- LaserScan topic exists and callbacks fire.
- Command velocity publishing works or command interface is available.
- Camera/YOLO/QR path is either live or explicitly degraded.
- Telemetry/log path is available if configured.

If LiDAR is missing or stale, do not move.

#### NAVIGATE

Default behavior.

Use LiDAR wall/corridor following at slow speed. Apply emergency stop before publishing any motion.

#### SIGN_CANDIDATE

A sign has been seen but is not yet confirmed.

Confirm using:

```text
same class >= N frames within last M frames
confidence >= threshold
bbox area ratio >= threshold
sensor data fresh
not in cooldown
```

#### TURNING_LEFT / TURNING_RIGHT

Execute a 90-degree turn. Prefer odometry/yaw if accessible. If not, use:

```text
timed rough turn
+ LiDAR alignment correction
```

Emergency stop can interrupt this state.

#### ALIGNING_AFTER_TURN

Use LiDAR to reduce angular error relative to the new corridor/walls.

Possible alignment signals:

- front direction has sufficient open space
- left/right distances stabilize
- side wall estimate is roughly parallel
- yaw command becomes small for several cycles

#### QR_SCAN

Slow or stop. Confirm decoded QR content for multiple frames. Append to persistent log.

#### RECOVERY

Triggered when front is blocked, navigation has no valid command, or the robot appears stuck.

Use largest-free-sector / Follow-the-Gap-style recovery, not blind 180-degree turning as the first response.

#### EMERGENCY_STOP

Publish zero velocity. Remain stopped until safety condition clears or user intervenes.

## Behavior priority

The arbiter must resolve commands using:

```text
1. Emergency stop
2. Active maneuver completion
3. QR scanning/logging
4. Confirmed traffic sign command
5. Default navigation
6. Safe idle
```

Do not allow YOLO to bypass LiDAR safety.

## LiDAR sector extraction

Suggested sectors in robot frame:

```text
front_center: -10° to +10°
front:        -20° to +20°
front_left:   +20° to +70°
front_right:  -70° to -20°
left:         +70° to +110°
right:        -110° to -70°
rear_left:    +150° to +180°
rear_right:   -180° to -150°
```

For each sector compute:

```text
min_range
robust_min_range = percentile(valid_ranges, 10)
median_range
num_valid
```

Use robust values for decisions when possible.

## Emergency stop

Suggested first thresholds, to be tuned on robot:

```text
front_stop_distance_m = 0.28 to 0.35
front_slow_distance_m = 0.45 to 0.60
side_stop_distance_m  = 0.12 to 0.18
```

Rules:

```text
if LiDAR stale:
    stop

if front_center < front_stop_distance:
    linear_x = 0
    allow evasive yaw only if side clearance permits

if side too close and yaw would turn into that side:
    veto that yaw

if no valid LiDAR points:
    stop and report
```

## Default navigation algorithm

Start with corridor centering.

Inputs:

```text
left_distance
right_distance
front_distance
front_left_distance
front_right_distance
```

Controller:

```text
error = left_distance - right_distance
d_error = (error - prev_error) / dt
angular_z = -Kp * error - Kd * d_error
linear_x = base_speed
```

Use lower speed in narrow corridors.

Example starting values:

```text
base_speed = 0.08 to 0.14 m/s
narrow_speed = 0.04 to 0.08 m/s
max_yaw = 0.5 to 0.9 rad/s
Kp = 0.4 to 0.8
Kd = 0.02 to 0.08
```

Tune on the real robot.

If one side is missing or too open, switch to wall-follow mode:

```text
if following left wall:
    error = desired_wall_distance - left_distance
if following right wall:
    error = right_distance - desired_wall_distance
```

## Navigation module boundary

The default navigation behavior must be implemented behind a replaceable module interface. Do not hardcode wall-following, Follow-the-Gap, FOCM, or any other algorithm directly into the main controller.

The main controller owns:
- ROS/LiDAR/camera subscriptions
- YOLO/QR event handling
- safety arbitration
- active maneuver state
- command publishing
- telemetry/logging

The navigation module owns only:
- computing a suggested `(linear_x, angular_z)` from processed LiDAR/navigation observations
- returning debug information explaining its decision

The navigation module must not:
- publish `/cmd_vel` directly
- override emergency safety
- read YOLO detections directly
- read QR detections directly
- perform UDP telemetry directly except through debug fields returned to the controller

The controller must support selecting the navigation module by config/CLI, for example:

```bash
--nav-module wall_follow
--nav-module largest_gap
--nav-module focm
```

## Free-space recovery

Use this only when default navigation cannot proceed.

Core idea:

```text
1. preprocess LaserScan
2. mark blocked rays below safety distance
3. find contiguous safe angular gaps
4. choose best gap
5. rotate toward center of selected gap
6. move forward slowly only when front is safe
```

Prefer gaps that are:

- wide enough for robot width + margin
- close to forward direction
- not pointing into a detected side wall
- stable across multiple scans

If all gaps are poor:

```text
stop
rotate slowly toward the relatively most open side
report RECOVERY_NO_CLEAR_GAP
```

Do not use blind 180-degree turn as first response.

## Visual sign interface

The YOLO side should expose a latest state with fields similar to:

```json
{
  "timestamp": 0.0,
  "class_name": "left",
  "confidence": 0.91,
  "bbox": [x1, y1, x2, y2],
  "bbox_area_ratio": 0.08,
  "source": "win/yolo/recibidor.py"
}
```

The navigation side should treat this as stale if:

```text
now - timestamp > max_signal_age_s
```

Suggested initial values:

```text
max_signal_age_s = 0.5 to 1.0
confirm_window = 8 frames/events
confirm_count = 5
min_confidence = 0.65 to 0.80
min_area_ratio = tune from logs
cooldown_s = 2.0 to 4.0
```

## Sign commands

Mapping example:

```text
left_sign  -> TURNING_LEFT
right_sign -> TURNING_RIGHT
stop_sign  -> stop/slow behavior, if present in dataset
```

Execution logic:

```text
if confirmed LEFT:
    stop or slow
    verify left/turn clearance
    turn left
    align after turn
    cooldown
    NAVIGATE
```

If turn clearance is poor:

```text
wait briefly
or move forward slowly until turn point is safer
or enter RECOVERY
```

## QR behavior

QR detection should not be treated as best-effort only. It is part of scoring/evidence.

Recommended behavior:

```text
if QR visible or QR decode candidate:
    reduce speed
    optionally stop for stable decode
    require stable content for K frames
    append persistent log
    mark content as seen
```

Suggested persistent file:

```text
output/qr_log.jsonl
```

Suggested record:

```json
{
  "timestamp": "2026-07-04T00:00:00",
  "qr_content": "...",
  "state": "QR_SCAN",
  "front_distance_m": 0.72,
  "sign_state": "none",
  "source": "camera"
}
```

## Testing milestones

### Milestone 0 — no movement

- inspect repo state
- verify ROS environment
- verify LiDAR topic and callback
- verify camera topic if needed
- verify YOLO latest signal file
- verify QR decode/log path
- verify UDP diagnostics

### Milestone 1 — safety only

- run LiDAR node
- publish zero velocity by default
- approach obstacle by hand
- confirm stop reasons/logs

### Milestone 2 — slow navigation

- low-speed corridor centering
- emergency stop active
- no YOLO integration yet

### Milestone 3 — sign event integration

- feed synthetic/latest-signal JSON
- ensure debouncing/cooldown
- test turn command in open space

### Milestone 4 — real visual integration

- run YOLO receiver
- confirm sign events
- robot executes one left/right command safely

### Milestone 5 — QR evidence

- QR detection triggers slow/stop
- log persists
- repeated QR is not duplicated

### Milestone 6 — integrated circuit test

- run full behavior
- save logs
- record failures with timestamps and reasons
