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
  -> selected navigation module from wall_following.py
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
```

Safety, QR, YOLO reading, diagnostics, and command publishing stay outside the
navigation module.

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
  -p telemetry_port:=6001 \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl \
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

If LiDAR is missing, expected safe failure:

```text
state=SENSOR_CHECK reason=NO_LIDAR_SECTOR_MAP cmd=(0.000,0.000)
```

## UDP diagnostics

Use port `6001` for the new navigator so it does not collide with the existing
image telemetry sender on port `6000`.

Laptop receiver:

```powershell
$env:ROBOT_PORT=6001
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
  -p telemetry_port:=6001 \
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
  -p telemetry_port:=6001 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl \
  -p max_yaw:=0.35 \
  -p wall_kp:=0.25 \
  -p wall_kd:=0.02 \
  -p base_speed:=0.05 \
  -p narrow_speed:=0.03
```

Expected logs:

```text
dry_run=False enable_motion=True
state=CORRIDOR_FOLLOW
cmd=(0.050,...)
scan_count increasing
lidar_age < 0.5s
```

## If logs show non-zero cmd but robot does not move

Example:

```text
cmd=(0.050,0.073) dry_run=False enable_motion=True
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
