# Reactive Nav: What Runs on macOS vs on the TurtleBot

This is the operator guide for the stabilized reactive navigation stack.

The split is intentional:

- macOS runs the receivers and live camera view.
- the TurtleBot runs ROS 2, reactive navigation, safety arbitration, QR logging, and wheel commands.

## 1. What to run on your Mac

Run everything from the repo root:

```bash
cd <repo-root>
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

### Live robot camera window

This is the official live view:

```bash
export ROBOT_PORT=6610
python3 win/yolo/recibidor.py <robot_ip>
```

That window shows what the robot camera sees and, by default, also runs YOLO and writes:

```text
output/signals/latest_signal.json
```

If you only want the live camera without YOLO inference:

```bash
export ROBOT_PORT=6610
python3 win/yolo/recibidor.py <robot_ip> --view-only
```

### Nav state and stop-reason receiver

Run this in parallel to watch state transitions, stop reasons, and motion decisions:

```bash
export ROBOT_PORT=6612
python3 win/lidar/recibidor.py <robot_ip>
```

### Optional QR receiver

Only use this if you are running the separate QR telemetry sender:

```bash
export ROBOT_PORT=6611
python3 win/detect_qr/recibidor.py <robot_ip>
```

### Do not run this with the new stack

```bash
python3 win/yolo/enviador.py
```

That script sends wheel commands directly. The new architecture expects YOLO to publish symbolic sign state only.

## 2. What to run on the TurtleBot

Prepare the environment:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
```

The reactive navigator must be run on the TurtleBot.

## 3. Navigation module selection

You can switch the local navigation module explicitly with ROS parameters:

```text
-p nav_module:=wall_follow
-p nav_module:=follow_gap
-p nav_module:=focm
```

Named ROS profiles live in:

```text
ubuntu/reactive_nav/configs/
```

Current starter profiles:

```text
wall_follow_safe.yaml
wall_follow_fast.yaml
follow_gap_safe.yaml
focm_safe.yaml
```

## 4. Recommended no-motion validation

Start with dry-run mode so the robot does not move:

```bash
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_safe.yaml \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/collision_frames
```

While that is running, confirm on the Mac:

1. `win/yolo/recibidor.py` shows live camera frames.
2. `win/lidar/recibidor.py` shows state, reason, and command updates.
3. `output/signals/latest_signal.json` is updating when signs are visible.

## 5. Movement run with a named profile

Only run this when the robot is in a safe physical test area:

```bash
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_fast.yaml \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/collision_frames
```

## 6. Movement run with raw `-p` flags

Use this when you want to override a profile or test a module quickly:

```bash
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  -p profile_name:=manual_follow_gap_test \
  -p nav_module:=follow_gap \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p base_speed:=0.12 \
  -p narrow_speed:=0.05 \
  -p turn_slow_speed:=0.07 \
  -p front_stop_distance:=0.28 \
  -p front_stop_clear_distance:=0.38 \
  -p side_stop_distance:=0.12 \
  -p side_stop_clear_distance:=0.19 \
  -p emergency_clear_cycles:=3 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl
```

## 7. What the logs mean

Persistent run logs are written on the TurtleBot to:

```text
/home/ubuntu/output/reactive_nav_debug.jsonl
```

They now include:

- `profile_name`
- `nav.module`
- `turn.turn_phase`
- `turn.turn_direction`
- `turn.align_error`
- `turn.align_yaw_clamped`
- `turn.turn_completed_reason`
- `emergency.emergency_active`
- `emergency.emergency_trigger_reason`
- `emergency.emergency_clear_counter`
- `emergency.emergency_trigger_count`

Collision-triggered evidence is still written to:

```text
/home/ubuntu/output/collision_events.jsonl
/home/ubuntu/output/collision_frames/
```

## 8. Evaluation loop

Use the same flow for every profile:

1. run one named profile
2. collect `/home/ubuntu/output/reactive_nav_debug.jsonl`
3. copy it to the Mac
4. summarize it with the evaluator
5. compare the metrics across profiles

Example:

```bash
scp turtlebot4:/home/ubuntu/output/reactive_nav_debug.jsonl output/wall_follow_fast_run1.jsonl
python3 scripts/evaluate_nav_profiles.py output/wall_follow_fast_run1.jsonl
```

Compare multiple runs at once:

```bash
python3 scripts/evaluate_nav_profiles.py \
  output/wall_follow_fast_run1.jsonl \
  output/follow_gap_safe_run1.jsonl \
  output/focm_safe_run1.jsonl
```

Reported metrics:

- total runtime
- time spent in each state
- emergency-stop count
- emergency-stop total time
- average published linear speed during `CORRIDOR_FOLLOW`
- recovery time ratio
- turn count
- average turn completion time

## 9. If the robot does not move

If the navigator logs non-zero commands but the base does not move, run on the TurtleBot:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel --once
ros2 topic info /cmd_vel_unstamped --verbose
ros2 topic echo /cmd_vel_unstamped --once
```

Also check:

```bash
ros2 topic echo /hazard_detection --once
ros2 topic echo /wheel_status --once
ros2 topic echo /dock_status --once
ros2 topic echo /diagnostics --once
```

If motion is blocked, keep the JSONL log, collision log, and live camera view together. That is the minimum evidence set for debugging the next run.
