# Next Robot Dry-Run Protocol

## Purpose

After offline improvements, the next physical step should be dry-run/no-motion validation, not immediate movement.

Dry-run validation checks whether the tuned controller behaves sensibly with live sensors while publishing zero velocity.

## Preconditions

Before dry-run:

```text
unit tests pass
harsh synthetic benchmark passes safety cases
real-log replay improves or does not regress
ablation report identifies promoted changes
new tuned YAML exists
```

Recommended tuned profile:

```text
ubuntu/reactive_nav/configs/wall_follow_tuned.yaml
```

## Robot command

Run on TurtleBot:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_tuned.yaml \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/reactive_nav_debug_tuned_dryrun.jsonl \
  -p collision_log_path:=/home/ubuntu/output/collision_events_tuned_dryrun.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/collision_frames_tuned_dryrun
```

## What to watch live

On the Mac, run the diagnostics receiver:

```bash
export ROBOT_PORT=6612
python3 win/lidar/recibidor.py <robot_ip>
```

Check:

```text
scan_count increasing
lidar_age fresh
state stable in normal corridor
no repeated RECOVERY loops while stationary
suggested angular_z not saturated constantly
corner veto debug fields appear only when geometry warrants it
anti-spin does not trigger while robot is stationary in dry-run unless the simulated command pattern warrants it
stale YOLO ignored
QR path does not block normal navigation unless QR visible/decoded
```

## Copy logs back

```bash
scp turtlebot4:/home/ubuntu/output/reactive_nav_debug_tuned_dryrun.jsonl output/reactive_nav_debug_tuned_dryrun.jsonl
scp turtlebot4:/home/ubuntu/output/collision_events_tuned_dryrun.jsonl output/collision_events_tuned_dryrun.jsonl
```

Then run:

```bash
python3 scripts/analyze_robot_failure_log.py output/reactive_nav_debug_tuned_dryrun.jsonl
python3 scripts/compare_nav_profiles.py output/reactive_nav_debug_tuned_dryrun.jsonl
```

## Dry-run acceptance

Dry-run passes if:

```text
LiDAR callbacks fresh
camera callbacks fresh if QR enabled
no sensor-missing stop in normal conditions
no constant yaw saturation
no state flapping
no repeated recovery loops
corner/side vetoes trigger only in plausible risky geometry
logs contain enough debug fields to explain decisions
```

Dry-run does not prove physical movement readiness. It only proves that live sensor input is being processed safely.

## First movement after dry-run

Only after dry-run passes, do a very low-speed movement test in open space:

```text
base_speed <= 0.03 to 0.04 m/s
narrow_speed <= 0.02 to 0.025 m/s
max_yaw conservative
operator ready to stop robot
```

Record and preserve logs from this first movement test.
