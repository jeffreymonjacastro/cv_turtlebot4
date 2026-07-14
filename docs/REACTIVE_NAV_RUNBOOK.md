# Reactive Navigation Runbook

This document explains how to run and debug the new TurtleBot4 reactive
navigation stack:

```text
LiDAR safety/navigation
+ YOLO LEFT/RIGHT symbolic events
+ QR checkpoint logging
+ safety-first behavior arbiter
```

The robot-side implementation is in:

```text
ubuntu/reactive_nav/
```

The main entrypoint is:

```text
ubuntu/reactive_nav/reactive_navigator.py
```

## Mental model

The navigator is not a YOLO driver. YOLO never drives wheels directly.

Runtime flow:

```text
/scan
  -> lidar_sectors.py
  -> selected navigation module from wall_following.py / gap_navigation.py
  -> behavior_arbiter.py safety checks
  -> /cmd_vel TwistStamped
  -> create3_repub
  -> Create 3 base

/oakd/rgb/preview/image_raw
  -> local QR detector
  -> qr_logger.py
  -> output/qr_log.jsonl

output/signals/latest_signal.json
  -> symbolic YOLO event reader
  -> sign debounce/cooldown
  -> behavior_arbiter.py
```

The navigation algorithm is replaceable through a factory:

```bash
-p nav_module:=wall_follow
-p nav_module:=follow_gap
-p nav_module:=focm
```

Safety, QR, YOLO reading, diagnostics, and command publishing stay outside the
navigation module.

Available modules:

- `wall_follow`: corridor/wall following with free-gap recovery.
- `follow_gap`: F1TENTH-style Follow-the-Gap using nearest-obstacle bubble removal and largest safe angular gap selection.
- `focm`: Follow the Obstacle Circle Method using physical gap width selection and obstacle-circle tangent heading.

Shared gap/FOCM tuning parameters:

```bash
-p gap_bubble_radius_m:=0.30
-p gap_min_width_deg:=18.0
-p gap_search_min_deg:=-120.0
-p gap_search_max_deg:=120.0
-p robot_width_m:=0.36
-p gap_side_margin_m:=0.08
-p focm_alpha:=40.0
```

## Do not run these together

Do not run `win/yolo/enviador.py` while running `reactive_navigator.py`.

`win/yolo/enviador.py` reads YOLO state and sends wheel commands directly over
UDP. The new stack needs YOLO only as a symbolic latest-state source. The arbiter
must be the only module deciding motion.

It is fine to run:

```text
win/yolo/recibidor.py
```

because it receives images, runs YOLO, and writes `output/signals/latest_signal.json`.

## Robot setup

On the robot:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
```

If testing from the temporary deployment used during validation:

```bash
cd /home/ubuntu
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --self-test
```

Expected:

```text
reactive_nav self-test passed
```

## Required non-moving validation

Run these before enabling motion.

### ROS graph

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
ros2 node list
ros2 topic list -t
```

Required topics:

```text
/scan [sensor_msgs/msg/LaserScan]
/oakd/rgb/preview/image_raw [sensor_msgs/msg/Image]
/cmd_vel [geometry_msgs/msg/TwistStamped]
```

### LiDAR publisher and messages

```bash
ros2 topic info /scan --verbose
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Acceptance:

```text
publisher count > 0
message arrives
ranges length > 0
reasonable finite ranges exist
```

On the validated robot, `/scan` arrived at about `7.8 Hz`.

### Camera publisher and messages

```bash
ros2 topic info /oakd/rgb/preview/image_raw --verbose
ros2 topic hz /oakd/rgb/preview/image_raw
ros2 topic echo /oakd/rgb/preview/image_raw --once --field header
```

On the validated robot, RGB frames arrived at about `30 Hz`.

### Command interface

Check type and subscribers:

```bash
ros2 topic info /cmd_vel --verbose
```

Expected:

```text
Type: geometry_msgs/msg/TwistStamped
Subscription count: 1
Node name: create3_repub
```

Zero command only:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

This should not move the robot.

## Dry-run mode

Dry-run mode is the first integrated test. It computes commands and publishes
zero velocity.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p telemetry_port:=6612 \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/collision_frames \
  -p diagnostic_period_s:=1.0
```

Expected logs:

```text
[LIDAR] subscribed scan_topic=/scan publishers=1
[QR] subscribed image_topic=/oakd/rgb/preview/image_raw publishers=1
[STATE] state=CORRIDOR_FOLLOW ... dry_run=True enable_motion=False scan_count=...
```

Acceptance:

```text
scan_count increases
lidar_age stays below 0.5s
image_age stays fresh
dry_run=True
enable_motion=False
```

Persistent debug records are appended to:

```text
/home/ubuntu/output/reactive_nav_debug.jsonl
```

Each JSONL record includes:

```text
state / reason
scan_count and freshness
sector distances and valid counts
left_minus_right_m
navigation module error and d_error
suggested command
arbiter-requested command
actually published command
YOLO signal state
dry_run / enable_motion flags
```

For a left-turn bias, inspect:

```bash
tail -n 50 /home/ubuntu/output/reactive_nav_debug.jsonl
```

Key fields:

```text
lidar.left_minus_right_m
nav.debug.error
nav.debug.d_error
nav.suggested_angular_z
command.requested_angular_z
command.published_angular_z
nav.debug.yaw_veto
```

Copy the log to the Mac and summarize it:

```bash
scp turtlebot4:/home/ubuntu/output/reactive_nav_debug.jsonl output/reactive_nav_debug.jsonl
python3 scripts/analyze_reactive_nav_log.py output/reactive_nav_debug.jsonl
```

Create 3 hazard events are logged separately with a cooldown:

```text
/home/ubuntu/output/collision_events.jsonl
```

If the OAK-D image callback has a recent frame, the event also saves:

```text
/home/ubuntu/output/collision_frames/collision_<timestamp>.jpg
```

Copy event artifacts back to the Mac:

```bash
scp turtlebot4:/home/ubuntu/output/collision_events.jsonl output/collision_events.jsonl
scp -r turtlebot4:/home/ubuntu/output/collision_frames output/collision_frames
```

If LiDAR is missing, expected safe failure:

```text
state=SENSOR_CHECK reason=NO_LIDAR_SECTOR_MAP cmd=(0.000,0.000)
```

## UDP diagnostics

Use port `6612` for the new navigator diagnostics so it does not collide with
teammates using the legacy image telemetry sender on port `6000`.

Laptop receiver:

```powershell
$env:ROBOT_PORT=6612
python win/lidar/recibidor.py <robot_ip>
```

Expected packet types:

```text
ACK
LOG
LIDAR
```

The validated run received all three.

## YOLO setup

Laptop side:

```powershell
python win/yolo/recibidor.py
```

It writes:

```text
output/signals/latest_signal.json
```

The robot-side navigator must be able to read that file. During validation, the
robot read a synthetic file at:

```text
/home/ubuntu/output/signals/latest_signal.json
```

Run the navigator with:

```bash
-p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json
```

If the signal is missing or stale, logs show something like:

```text
signal=none/missing:output/signals/latest_signal.json
signal=left/stale:327.29s
```

That is safe. The robot ignores stale signs.

Synthetic LEFT dry-run test:

```bash
mkdir -p /home/ubuntu/output/signals
python3 -c 'import json,time,pathlib; p=pathlib.Path("/home/ubuntu/output/signals/latest_signal.json"); p.write_text(json.dumps({"direction":"left","confidence":0.95,"timestamp":time.time(),"bbox_area_ratio":0.20,"bbox_center_x_ratio":0.50,"actionable":True,"source_frame_time":"synthetic"}), encoding="utf-8")'

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p telemetry_port:=6612 \
  -p sign_confirm_window:=1 \
  -p sign_confirm_count:=1 \
  -p max_signal_age_s:=30.0 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json
```

Expected:

```text
state=TURNING_LEFT reason=TIMED_90_DEGREE_TURN dry_run=True enable_motion=False
```

## QR logging

Default robot log path:

```text
output/qr_log.jsonl
```

Recommended robot path:

```bash
-p qr_log_path:=/home/ubuntu/output/qr_log.jsonl
```

Check:

```bash
tail -f /home/ubuntu/output/qr_log.jsonl
```

Validation behavior:

```text
first same QR sighting: wait for confirmation
second same QR sighting: append JSONL record
later same content: duplicate ignored
```

## First movement command

Only run this in open space with someone ready to stop the robot.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/collision_frames \
  -p collision_cooldown_s:=2.0 \
  -p max_yaw:=0.65 \
  -p wall_kp:=0.45 \
  -p wall_kd:=0.03 \
  -p base_speed:=0.04 \
  -p narrow_speed:=0.025 \
  -p front_clear_distance:=0.70 \
  -p front_corner_avoid_distance:=0.70 \
  -p side_avoid_distance:=0.38 \
  -p avoidance_gain:=0.85
```

Expected logs:

```text
dry_run=False enable_motion=True
state=CORRIDOR_FOLLOW
cmd=(0.040,...)
scan_count increasing
lidar_age < 0.5s
```

## Real-movement test for `wall_follow_less_conservative`

Use this only after the dry-run gates pass. This profile is intentionally less
conservative than `wall_follow_tuned`, so treat it as a measured candidate, not
as the new default.

Required gates before motion:

```bash
cd /home/ubuntu/reactive_nav_test
bash scripts/run_turn_recovery_capture.sh angle_offset_dryrun \
  --profile-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  --duration-sec 20 \
  --no-bag
```

Acceptance before continuing:

```text
fresh scan callbacks
dry_run=True
enable_motion=False
front/left/right sectors look physically sane
no emergency-stop burst caused by obviously wrong angle offset
```

The movement tests below save each run under:

```text
/home/ubuntu/output/robot_runs/<timestamp>_wall_follow_less_conservative_<scenario>/
```

Each run directory includes:

```text
reactive_nav_debug.jsonl     primary evidence for extraction/replay/ablation
collision_events.jsonl       Create 3 hazard/collision evidence, if any
collision_frames/            camera frames near collision events, if available
profile.yaml                 exact profile used for the run
operator_note.md             human observation template
bag/                         optional ROS bag when --bag is used
```

### Candidate left-turn movement capture

Put the robot in open space with a safe left-turn corridor/aisle, and keep a
human ready to stop it. This command injects a synthetic LEFT sign by default so
the test actually exercises `TURNING_LEFT` instead of depending on YOLO timing.

```bash
cd /home/ubuntu/reactive_nav_test
bash scripts/run_turn_recovery_capture.sh left_turn \
  --profile-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  --duration-sec 20 \
  --no-bag
```

Expected evidence:

```text
state sequence includes TURNING_LEFT and/or ALIGNING_AFTER_TURN
published commands are non-zero only while dry_run=False enable_motion=True
emergency stop still interrupts if front/side clearance becomes unsafe
run directory printed at the end
```

### Candidate right-turn movement capture

```bash
cd /home/ubuntu/reactive_nav_test
bash scripts/run_turn_recovery_capture.sh right_turn \
  --profile-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  --duration-sec 20 \
  --no-bag
```

Expected evidence:

```text
state sequence includes TURNING_RIGHT and/or ALIGNING_AFTER_TURN
no repeated turn/recovery loop
no corner/side scrape intervention
run directory printed at the end
```

### Candidate front-blocked recovery movement capture

Use a controlled front-blocked setup with visible escape room to at least one
side. Do not box the robot into a real collision. This test is meant to verify
that recovery can rotate/select a gap and exit instead of freezing in place.

```bash
cd /home/ubuntu/reactive_nav_test
bash scripts/run_turn_recovery_capture.sh front_blocked_recovery \
  --profile-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  --duration-sec 30 \
  --no-bag
```

Expected evidence:

```text
state sequence may enter RECOVERY or FRONT_BLOCKED_SELECT_FREE_GAP
recovery should publish a turn command when there is a safe open side
recovery should exit when front clearance becomes safe
no long stationary loop with front blocked and zero yaw
```

### Optional bag recording

Use `--bag` for the best replay evidence if disk space allows:

```bash
bash scripts/run_turn_recovery_capture.sh left_turn \
  --profile-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  --duration-sec 20 \
  --bag
```

The JSONL log is still the primary input for the current extraction/replay
pipeline. The bag is extra evidence for later scan-level reconstruction.

### Pull movement evidence back to the Mac

From the Mac:

```bash
mkdir -p output/robot_runs
rsync -av turtlebot4:/home/ubuntu/output/robot_runs/ output/robot_runs/
```

If you only want the new candidate runs:

```bash
mkdir -p output/robot_runs
rsync -av --include='*/' --include='*wall_follow_less_conservative*/***' --exclude='*' \
  turtlebot4:/home/ubuntu/output/robot_runs/ \
  output/robot_runs/
```

### Analyze movement logs for replay/ablation

Run these locally after rsync:

```bash
.venv/bin/python -m pytest tests/

.venv/bin/python scripts/extract_turn_recovery_intervals.py \
  output/robot_runs/*wall_follow_less_conservative*/reactive_nav_debug.jsonl \
  --out-dir output/turn_recovery_analysis/wall_follow_less_conservative_real_movement

.venv/bin/python scripts/replay_turn_recovery_intervals.py \
  --intervals output/turn_recovery_analysis/wall_follow_less_conservative_real_movement/failure_intervals.jsonl \
  --profiles wall_follow_tuned wall_follow_less_conservative \
  --out-dir output/turn_recovery_replay/wall_follow_less_conservative_real_movement

.venv/bin/python scripts/run_turn_recovery_ablation.py \
  --intervals output/turn_recovery_analysis/wall_follow_less_conservative_real_movement/failure_intervals.jsonl \
  --out-dir output/turn_recovery_ablation/wall_follow_less_conservative_real_movement
```

If `failure_intervals.jsonl` is empty, keep the raw `reactive_nav_debug.jsonl`
runs anyway. They are still useful as passing movement evidence and can be used
to verify that future controller changes do not regress turn entry, recovery
exit, or safety veto behavior.

### Promotion rule

Do not replace `wall_follow_tuned` from one physical run. Promote the candidate
only if the pulled logs show:

```text
left and right movement captures enter turn states when forced signs are fresh
front-blocked recovery does not stay frozen with zero yaw
no increase in corner/side risk relative to baseline replay
no repeated turn/recovery state loop
no emergency-stop burst caused by relaxed thresholds
offline replay/ablation confirms the improvement source
```

## If logs show non-zero cmd but robot does not move

Example:

```text
cmd=(0.040,0.073) dry_run=False enable_motion=True
```

This means the navigator is computing and publishing motion. The problem is
downstream of the arbiter.

### 1. Confirm the navigator is a `/cmd_vel` publisher

While the navigator is running:

```bash
ros2 topic info /cmd_vel --verbose
```

Expected:

```text
Publisher count includes reactive_yolo_lidar_navigator
Subscription count includes create3_repub
```

If `reactive_yolo_lidar_navigator` is not listed, the node is not publishing to
the topic you think it is. Check:

```bash
-p cmd_topic:=/cmd_vel
-p cmd_msg_type:=TwistStamped
```

### 2. Confirm `/cmd_vel` actually contains non-zero messages

While the navigator is running:

```bash
ros2 topic echo /cmd_vel --once
```

Expected:

```text
twist:
  linear:
    x: 0.05
  angular:
    z: ...
```

If this shows zero while the navigator logs non-zero, another publisher may be
overriding or your echo sampled the wrong instant. Stop other command publishers
and retry.

Known command publishers seen during validation:

```text
teleop_twist_joy_node
udp_teleop_receiver
reactive_yolo_lidar_navigator
```

For autonomous testing, stop joystick/UDP teleop if they are publishing zeros
over the same topic.

### 3. Confirm `create3_repub` is forwarding to `/cmd_vel_unstamped`

```bash
ros2 topic info /cmd_vel_unstamped --verbose
ros2 topic echo /cmd_vel_unstamped --once
```

If `/cmd_vel` is non-zero but `/cmd_vel_unstamped` stays zero or silent, the
bridge from TurtleBot4 ROS to Create 3 is the blockage.

### 4. Check Create 3 safety and mobility state

```bash
ros2 topic echo /hazard_detection --once
ros2 topic echo /wheel_status --once
ros2 topic echo /dock_status --once
ros2 topic echo /interface_buttons --once
ros2 topic echo /diagnostics --once
```

Look for:

```text
cliff / bump / wheel drop hazards
robot docked
estop / stop button state
base not ready
diagnostic errors
```

If the Create 3 base is refusing motion, the navigator can publish correctly and
still not move the wheels.

### 5. Confirm a direct minimal command works

Only do this when the robot is in open space. This physically moves the robot.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.03}, angular: {z: 0.0}}}"
```

Then immediately stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

If direct publish does not move the robot, the issue is not the navigator. It is
the robot command path, Create 3 state, safety state, or bringup configuration.

### 6. Try `/cmd_vel_unstamped` only if needed

If `/cmd_vel` does not drive the base but `/cmd_vel_unstamped` has subscribers,
test cautiously:

```bash
ros2 topic info /cmd_vel_unstamped --verbose
```

Only in open space:

```bash
ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.03}, angular: {z: 0.0}}"
```

Then immediately stop:

```bash
ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

If this works and `/cmd_vel` does not, run the navigator with:

```bash
-p cmd_topic:=/cmd_vel_unstamped -p cmd_msg_type:=Twist
```

## Acceptance checklist

Minimum before a real circuit attempt:

```text
LiDAR callbacks fresh
camera callbacks fresh
dry-run state/logs visible
UDP diagnostics visible
/cmd_vel path confirmed to physically move with direct low-speed test
reactive navigator low-speed movement confirmed
stale YOLO ignored
fresh synthetic LEFT/RIGHT causes one maneuver
QR logs once and ignores duplicate content
```
