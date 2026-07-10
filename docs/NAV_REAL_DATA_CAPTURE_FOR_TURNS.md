# NAV_REAL_DATA_CAPTURE_FOR_TURNS.md — Capturing Useful Robot Data for Turn/Recovery Failures

## Purpose

Physical robot time should produce reusable evidence. Every failed turn should become a replayable or analyzable regression case.

Minimum evidence per run:

```text
reactive_nav_debug.jsonl
collision_events.jsonl, if present
qr_log.jsonl, if relevant
profile/config YAML used
human observation note
optional rosbag with /scan and command topics
```

---

## Robot setup

Run these commands on the TurtleBot before starting a capture session:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=2
cd /home/ubuntu
mkdir -p /home/ubuntu/output/robot_runs
```

If the robot workspace is mounted at a different path, keep the same command shape but update `/home/ubuntu/reactive_nav_test` to the local checkout path.

---

## Run directory convention

Use one folder per attempt:

```text
output/robot_runs/YYYYMMDD_HHMM_profile_scenario_result/
```

Examples:

```text
output/robot_runs/20260709_2130_wall_follow_tuned_left_turn_stuck_recovery/
output/robot_runs/20260709_2145_wall_follow_tuned_right_turn_front_blocked/
output/robot_runs/20260709_2205_follow_gap_safe_front_blocked_no_exit/
```

Each folder should contain:

```text
reactive_nav_debug.jsonl
collision_events.jsonl              # if available
qr_log.jsonl                        # if relevant
profile.yaml
operator_note.md
rosbag/                             # optional but preferred
summary_after_analysis.md           # generated later
```

---

## Capture command templates

Use `wall_follow_tuned.yaml` as the starting profile for turn/recovery evidence.

## Fast helper

For faster testing, use the shell helper instead of assembling the command by hand:

```bash
scripts/run_turn_recovery_capture.sh angle_offset_dryrun
scripts/run_turn_recovery_capture.sh left_turn
scripts/run_turn_recovery_capture.sh right_turn
scripts/run_turn_recovery_capture.sh front_blocked_recovery
```

Add `--no-bag` if you only want the live robot log and not a rosbag, or `--run-name <name>` if you want the folder name to be deterministic.

### 1. Angle-offset dry-run checks

Run this first to verify the live sensors, logging, and sector alignment without moving the robot:

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_angle_offset_dryrun
mkdir -p /home/ubuntu/output/robot_runs/$RUN_NAME

python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_tuned.yaml \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_frames
```

In another terminal, if you want a raw scan/velocity trace for later replay, record a bag while the dry-run is active:

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_angle_offset_dryrun
ros2 bag record \
  /scan \
  /cmd_vel \
  /hazard_detection \
  -o /home/ubuntu/output/robot_runs/$RUN_NAME/bag
```

Use this dry-run for the three angle-offset evidence passes:

```text
front_obstacle_check
left_obstacle_check
right_obstacle_check
```

Keep the robot stationary or hand-positioned so each check only validates the sensor geometry and logging, not motion behavior.

### 2. Isolated left-turn capture

Use this when you want a minimal real turn interval:

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_left_turn_capture
mkdir -p /home/ubuntu/output/robot_runs/$RUN_NAME

python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_tuned.yaml \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_frames
```

Record a bag in parallel if possible:

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_left_turn_capture
ros2 bag record \
  /scan \
  /cmd_vel \
  /hazard_detection \
  -o /home/ubuntu/output/robot_runs/$RUN_NAME/bag
```

Before starting the run, position the robot so the intended left turn is isolated from other obstacles.

### 3. Isolated right-turn capture

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_right_turn_capture
mkdir -p /home/ubuntu/output/robot_runs/$RUN_NAME

python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_tuned.yaml \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_frames
```

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_right_turn_capture
ros2 bag record \
  /scan \
  /cmd_vel \
  /hazard_detection \
  -o /home/ubuntu/output/robot_runs/$RUN_NAME/bag
```

### 4. Front-blocked recovery capture

Use this in a corridor or obstacle setup that should force recovery without a turn trigger:

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_front_blocked_recovery
mkdir -p /home/ubuntu/output/robot_runs/$RUN_NAME

python3 -B /home/ubuntu/reactive_nav_test/reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_tuned.yaml \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:=/home/ubuntu/output/qr_log.jsonl \
  -p persistent_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/reactive_nav_debug.jsonl \
  -p collision_log_path:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_events.jsonl \
  -p collision_image_dir:=/home/ubuntu/output/robot_runs/$RUN_NAME/collision_frames
```

```bash
export RUN_NAME=YYYYMMDD_HHMM_wall_follow_tuned_front_blocked_recovery
ros2 bag record \
  /scan \
  /cmd_vel \
  /hazard_detection \
  -o /home/ubuntu/output/robot_runs/$RUN_NAME/bag
```

---

## Operator note template

Create `operator_note.md` after each physical attempt:

```md
# Operator note

Profile:
Scenario tested:
Start pose / environment:
Expected behavior:
Observed behavior:
Approximate failure time:
Did it enter recovery?
Did it spin/circle?
Did it scrape/hit corner?
Was e-stop/manual intervention needed?
Any visible LiDAR/camera issue:
```

This human note is important because logs alone may not reveal that the robot physically scraped or appeared stuck.

---

## Recommended ROS bag recording

When possible, record raw scan and command-related topics:

```bash
ros2 bag record \
  /scan \
  /cmd_vel \
  /cmd_vel_unstamped \
  /hazard_detection \
  /wheel_status \
  /diagnostics \
  -o bags/<run_name>
```

If this is too heavy, minimally record:

```bash
ros2 bag record /scan /cmd_vel /hazard_detection -o bags/<run_name>
```

This enables later replay against modified controllers.

---

## Targeted robot tests

Do short, isolated tests before full circuit attempts:

```text
left_turn_only
right_turn_only
front_blocked_recovery_only
narrow_corridor_only
angle_offset_front_obstacle_check
angle_offset_left_obstacle_check
angle_offset_right_obstacle_check
```

Avoid full-course testing until these pass.

---

## Copy-back example

After a run:

```bash
mkdir -p output/robot_runs/<run_name>
scp turtlebot4:/home/ubuntu/output/reactive_nav_debug.jsonl output/robot_runs/<run_name>/reactive_nav_debug.jsonl
scp turtlebot4:/home/ubuntu/output/collision_events.jsonl output/robot_runs/<run_name>/collision_events.jsonl || true
scp turtlebot4:/home/ubuntu/output/qr_log.jsonl output/robot_runs/<run_name>/qr_log.jsonl || true
scp -r turtlebot4:/home/ubuntu/output/robot_runs/<run_name>/bag output/robot_runs/<run_name>/bag || true
cp ubuntu/reactive_nav/configs/wall_follow_tuned.yaml output/robot_runs/<run_name>/profile.yaml
```

Then run:

```bash
python3 scripts/analyze_robot_failure_log.py output/robot_runs/<run_name>/reactive_nav_debug.jsonl \
  --out-dir output/robot_runs/<run_name>/analysis
```
