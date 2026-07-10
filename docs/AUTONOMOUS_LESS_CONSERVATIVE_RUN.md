# Autonomous Run With `wall_follow_less_conservative`

This is the direct run recipe for the main autonomous task using:

```text
LiDAR reactive navigation
+ laptop YOLO symbolic signs
+ laptop ZXing QR checkpoints
+ robot priority-based behavior arbiter
+ wall_follow_less_conservative profile
```

This is not a benchmark command. The final navigator command enables robot
motion.

The arbiter priority remains:

```text
1. Emergency LiDAR stop / collision prevention
2. Active maneuver completion, unless emergency stop is needed
3. QR scan/checkpoint behavior
4. Confirmed YOLO traffic-sign command
5. Default LiDAR navigation
6. Stop if required sensors are missing or stale
```

Do not run `win/yolo/enviador.py` for this stack. That legacy script sends
wheel commands directly. Here, YOLO and QR only write semantic JSON files; the
robot-side arbiter owns motion.

## 0. Sync Code to the Robot

Run on the Mac:

```bash
cd /Users/katharsis/Developer/cv/turtle4
rsync -av ubuntu/reactive_nav/ turtlebot4:/home/ubuntu/reactive_nav_test/reactive_nav/
rsync -av docs/AUTONOMOUS_LESS_CONSERVATIVE_RUN.md turtlebot4:/home/ubuntu/reactive_nav_test/docs/AUTONOMOUS_LESS_CONSERVATIVE_RUN.md
```

## 1. Robot Terminal A: Stream Camera Frames

```bash
ssh turtlebot4
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

Leave this running. It waits for the laptop receiver to send `HELLO`, then
streams JPEG frames over UDP. It never publishes motion commands.

## 2. Laptop Terminal B: Run YOLO + ZXing QR

```bash
cd /Users/katharsis/Developer/cv/turtle4
RUN_ID="$(date +%Y%m%d_%H%M%S)_wall_follow_less_conservative_autonomous"
mkdir -p "output/perception_runs/$RUN_ID"

uv run python win/yolo/recibidor.py \
  --enable-qr \
  --qr-event-path output/signals/latest_qr_event.json \
  --perception-log "output/perception_runs/$RUN_ID/laptop_perception.jsonl"
```

This writes:

```text
output/signals/latest_signal.json
output/signals/latest_qr_event.json
output/perception_runs/<run_id>/laptop_perception.jsonl
```

The camera overlay shows YOLO boxes plus QR status, payload, confirmation
progress, decode variant, latency, and queue metrics.

## 3. Laptop Terminal C: Sync YOLO and QR to the Robot

```bash
cd /Users/katharsis/Developer/cv/turtle4
RUN_ID="$(ls -td output/perception_runs/*_wall_follow_less_conservative_autonomous | head -n 1 | xargs basename)"

uv run python win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --qr-source output/signals/latest_qr_event.json \
  --qr-remote-path /home/ubuntu/output/signals/latest_qr_event.json \
  --interval 0.2 \
  --log-path "output/perception_runs/$RUN_ID/sync.jsonl"
```

This helper copies through temporary remote files and atomically renames into
place. The robot reads:

```text
/home/ubuntu/output/signals/latest_signal.json
/home/ubuntu/output/signals/latest_qr_event.json
```

## 4. Robot Terminal D: Run Autonomous Navigation With Motion Enabled

Only run this after the test gates in `docs/QR_ZXING_TEST_RUN.md` have passed.

```bash
ssh turtlebot4
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
  -p enable_qr_events:=true \
  -p enable_external_qr_events:=true \
  -p enable_qr_detection:=false \
  -p qr_injection_path:=/home/ubuntu/output/signals/latest_qr_event.json \
  -p max_external_qr_source_frame_age_s:=0.5 \
  -p qr_log_path:="$RUN_DIR/qr_log.jsonl" \
  -p persistent_log_path:="$RUN_DIR/reactive_nav_debug.jsonl" \
  -p collision_log_path:="$RUN_DIR/collision_events.jsonl" \
  -p collision_image_dir:="$RUN_DIR/collision_frames"
```

Stop with `Ctrl+C`.

Robot run directory:

```text
/home/ubuntu/output/autonomous_runs/<timestamp>_wall_follow_less_conservative_autonomous/
```

Persistent QR scan information is saved automatically in:

```text
/home/ubuntu/output/autonomous_runs/<run_id>/qr_log.jsonl
```

Each QR record includes normalized content, raw content, event ID, source frame,
barcode format, decode variant, decode latency, robot state, and LiDAR context.
The matching FSM/command evidence is in:

```text
/home/ubuntu/output/autonomous_runs/<run_id>/reactive_nav_debug.jsonl
```

## 5. Optional Laptop Diagnostics

```bash
cd /Users/katharsis/Developer/cv/turtle4
python3 win/lidar/recibidor.py <robot_ip>
```

The navigator sends diagnostics on UDP port `6612`.

## 6. Pull Logs Back

Run on the Mac after the run:

```bash
cd /Users/katharsis/Developer/cv/turtle4
mkdir -p output/autonomous_runs
rsync -av turtlebot4:/home/ubuntu/output/autonomous_runs/ output/autonomous_runs/
```

Keep the paired laptop evidence in:

```text
output/perception_runs/<run_id>/laptop_perception.jsonl
output/perception_runs/<run_id>/sync.jsonl
```

## 7. Later: Turn/Recovery Replay and Ablation

```bash
cd /Users/katharsis/Developer/cv/turtle4

uv run python scripts/extract_turn_recovery_intervals.py \
  output/autonomous_runs/*wall_follow_less_conservative_autonomous/reactive_nav_debug.jsonl \
  --out-dir output/turn_recovery_analysis/wall_follow_less_conservative_autonomous

uv run python scripts/replay_turn_recovery_intervals.py \
  --intervals output/turn_recovery_analysis/wall_follow_less_conservative_autonomous/failure_intervals.jsonl \
  --profiles wall_follow_tuned wall_follow_less_conservative \
  --out-dir output/turn_recovery_replay/wall_follow_less_conservative_autonomous

uv run python scripts/run_turn_recovery_ablation.py \
  --intervals output/turn_recovery_analysis/wall_follow_less_conservative_autonomous/failure_intervals.jsonl \
  --out-dir output/turn_recovery_ablation/wall_follow_less_conservative_autonomous
```

If no failure intervals are extracted, keep the run anyway. A clean autonomous
run is still useful regression evidence.

## Quick Stop

In the robot navigation terminal, press:

```text
Ctrl+C
```

To force a zero command:

```bash
ssh turtlebot4
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

## QR Rollback

Keep laptop YOLO but disable laptop QR by omitting `--enable-qr` and omitting
the `--qr-source` sync flags.

Restore robot-side OpenCV QR fallback by running the navigator with:

```text
enable_qr_events=true
enable_external_qr_events=false
enable_qr_detection=true
qr_injection_path=output/qr_injection.json
```
