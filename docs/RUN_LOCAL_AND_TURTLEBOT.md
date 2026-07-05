# What To Run Locally vs On The TurtleBot

This is the short operator checklist for the reactive navigation stack.

## Local Computer (macOS)

Run these commands from the repo root on your Mac:

```bash
cd /Users/katharsis/Developer/cv/turtle4
```

Create and activate a local virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

Run the diagnostics receiver:

```bash
export ROBOT_PORT=6001
python3 win/lidar/recibidor.py <robot_ip>
```

Run the YOLO detector/receiver:

```bash
python3 win/yolo/recibidor.py
```

This script opens an OpenCV display window, so run it from the Mac desktop
session, not from an SSH session into the TurtleBot.

Do not run this with the new navigator:

```bash
python3 win/yolo/enviador.py
```

`win/yolo/enviador.py` sends wheel commands directly. The new navigation stack
expects YOLO to write symbolic state only, and the robot-side arbiter decides
movement.

## TurtleBot

Prepare the ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
```

Run dry-run mode first. This verifies sensors, diagnostics, YOLO state reading,
QR logging setup, and command decisions without moving the robot.

```bash
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p telemetry_port:=6001 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug.jsonl
```

Only when the robot is in open space and ready for movement, run:

```bash
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

Expected movement-mode logs:

```text
dry_run=False enable_motion=True
state=CORRIDOR_FOLLOW
cmd=(0.050,...)
scan_count increasing
lidar_age < 0.5s
```

Persistent debug records are written on the TurtleBot to:

```bash
/home/ubuntu/output/reactive_nav_debug.jsonl
```

After a run, inspect the latest records:

```bash
tail -n 20 /home/ubuntu/output/reactive_nav_debug.jsonl
```

For left/right bias, look at:

```text
lidar.left_minus_right_m
nav.debug.error
nav.debug.d_error
nav.suggested_angular_z
command.requested_angular_z
command.published_angular_z
nav.debug.yaw_veto
```

To copy the log back to your Mac and summarize it:

```bash
scp turtlebot4:/home/ubuntu/output/reactive_nav_debug.jsonl output/reactive_nav_debug.jsonl
python3 scripts/analyze_reactive_nav_log.py output/reactive_nav_debug.jsonl
```

## If Commands Are Logged But The Robot Does Not Move

On the TurtleBot, while the navigator is running:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel --once
ros2 topic info /cmd_vel_unstamped --verbose
ros2 topic echo /cmd_vel_unstamped --once
```

Interpretation:

```text
/cmd_vel non-zero, /cmd_vel_unstamped zero or silent:
  create3_repub or the TurtleBot command bridge is blocking motion.

/cmd_vel zero, navigator logs non-zero:
  another publisher may be overriding, or the navigator is not publishing to
  the expected topic.

/cmd_vel_unstamped non-zero, robot still stationary:
  check Create 3 safety/mobility state.
```

Useful safety/mobility checks:

```bash
ros2 topic echo /hazard_detection --once
ros2 topic echo /wheel_status --once
ros2 topic echo /dock_status --once
ros2 topic echo /interface_buttons --once
ros2 topic echo /diagnostics --once
```
