# Autonomous run with `wall_follow_less_conservative`

This is the direct run recipe for the main autonomous task using:

```text
LiDAR reactive navigation
+ YOLO LEFT/RIGHT symbolic events
+ QR detection/logging
+ priority-based behavior arbiter
+ wall_follow_less_conservative profile
```

This is not a dry-run or benchmark command. It enables robot motion.

The arbiter priority remains:

```text
1. Emergency LiDAR stop / collision prevention
2. Active maneuver completion, unless emergency stop is needed
3. QR scan/checkpoint behavior
4. Confirmed YOLO traffic-sign command
5. Default LiDAR navigation
6. Stop if required sensors are missing or stale
```

Do not run `win/yolo/enviador.py` for this mode. That script sends wheel
commands directly. For this stack, YOLO only writes `latest_signal.json`; the
robot-side arbiter decides motion.

## 0. Sync the latest robot-side code

From the Mac/local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4
rsync -av ubuntu/reactive_nav/ turtlebot4:/home/ubuntu/reactive_nav_test/reactive_nav/
rsync -av docs/AUTONOMOUS_LESS_CONSERVATIVE_RUN.md turtlebot4:/home/ubuntu/reactive_nav_test/docs/AUTONOMOUS_LESS_CONSERVATIVE_RUN.md
```

## 1. Robot terminal A: stream camera frames to the laptop YOLO receiver

Open an SSH terminal to the robot:

```bash
ssh turtlebot4
```

Run:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

cd /home/ubuntu/reactive_nav_test
python3 -B reactive_nav/debug_image_udp_sender.py --ros-args \
  -p port:=6610 \
  -p image_topic:=/oakd/rgb/preview/image_raw \
  -p send_hz:=5.0 \
  -p jpeg_quality:=80
```

Leave this running. It waits for the laptop YOLO receiver to send a `HELLO`,
then streams camera frames over UDP. It never publishes motion commands.

## 2. Laptop terminal B: run YOLO receiver

From the local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4
python3 win/yolo/recibidor.py
```

This writes:

```text
output/signals/latest_signal.json
```

That file is the only YOLO output used by the autonomous navigator.

## 3. Laptop terminal C: sync YOLO latest signal to the robot

From the local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4
python3 win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --interval 0.2
```

Leave this running. The robot navigator reads:

```text
/home/ubuntu/output/signals/latest_signal.json
```

If YOLO is missing, stale, or uncertain, the arbiter ignores it and continues
with LiDAR navigation.

## 4. Robot terminal D: run autonomous navigation with motion enabled

Open another SSH terminal to the robot:

```bash
ssh turtlebot4
```

Run:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

RUN_ID="$(date +%Y%m%d_%H%M%S)_wall_follow_less_conservative_autonomous"
RUN_DIR="/home/ubuntu/output/autonomous_runs/$RUN_ID"
mkdir -p "$RUN_DIR/collision_frames"
cp /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml "$RUN_DIR/profile.yaml"

cd /home/ubuntu/reactive_nav_test
python3 -B reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  -p dry_run:=false \
  -p enable_motion:=true \
  -p telemetry_port:=6612 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_log_path:="$RUN_DIR/qr_log.jsonl" \
  -p persistent_log_path:="$RUN_DIR/reactive_nav_debug.jsonl" \
  -p collision_log_path:="$RUN_DIR/collision_events.jsonl" \
  -p collision_image_dir:="$RUN_DIR/collision_frames"
```

This starts the actual autonomous run. Stop it with `Ctrl+C`.

The run directory is:

```text
/home/ubuntu/output/autonomous_runs/<timestamp>_wall_follow_less_conservative_autonomous/
```

It contains:

```text
profile.yaml                 exact profile used
reactive_nav_debug.jsonl     full arbiter/navigation/sector/command evidence
qr_log.jsonl                 QR checkpoint evidence
collision_events.jsonl       Create 3 hazard/collision evidence, if any
collision_frames/            camera frames near collision events, if available
```

## 5. Optional laptop diagnostics terminal

This is optional, but useful while the robot is moving:

```bash
cd /Users/katharsis/Developer/cv/turtle4
python3 win/lidar/recibidor.py <robot_ip>
```

The navigator sends diagnostics on UDP port `6612`.

## 6. Pull the run logs back after autonomous driving

From the Mac/local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4
mkdir -p output/autonomous_runs
rsync -av turtlebot4:/home/ubuntu/output/autonomous_runs/ output/autonomous_runs/
```

The most important file for later ablation/synthetic replay is:

```text
output/autonomous_runs/<run_id>/reactive_nav_debug.jsonl
```

## 7. Later: convert the run into replay/ablation evidence

After pulling logs locally:

```bash
cd /Users/katharsis/Developer/cv/turtle4

.venv/bin/python scripts/extract_turn_recovery_intervals.py \
  output/autonomous_runs/*wall_follow_less_conservative_autonomous/reactive_nav_debug.jsonl \
  --out-dir output/turn_recovery_analysis/wall_follow_less_conservative_autonomous

.venv/bin/python scripts/replay_turn_recovery_intervals.py \
  --intervals output/turn_recovery_analysis/wall_follow_less_conservative_autonomous/failure_intervals.jsonl \
  --profiles wall_follow_tuned wall_follow_less_conservative \
  --out-dir output/turn_recovery_replay/wall_follow_less_conservative_autonomous

.venv/bin/python scripts/run_turn_recovery_ablation.py \
  --intervals output/turn_recovery_analysis/wall_follow_less_conservative_autonomous/failure_intervals.jsonl \
  --out-dir output/turn_recovery_ablation/wall_follow_less_conservative_autonomous
```

If no failure intervals are extracted, keep the run anyway. A clean autonomous
run is still useful evidence for future regression checks.

## Quick stop

In the robot navigation terminal, press:

```text
Ctrl+C
```

If you need to force a zero command:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```
