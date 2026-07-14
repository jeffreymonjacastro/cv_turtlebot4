# ZXing QR Test Run

This validates the laptop ZXing QR path without physical motion:

```text
robot camera
-> laptop ZXing decode
-> laptop validation
-> latest_qr_event.json
-> atomic robot sync
-> robot receipt
-> QRLogger checkpoint evidence
-> arbiter/FSM diagnostics
-> published command remains zero in dry-run
```

## 0. Local Dependency Check

Run on the Mac:

```bash
cd <repo-root>
uv sync --locked
uv run python -c "import importlib.metadata, zxingcpp; print(importlib.metadata.version('zxing-cpp'), zxingcpp.__file__)"
uv run python -m pytest tests/
uv run python -B ubuntu/reactive_nav/reactive_navigator.py --self-test
```

## 1. Sync Robot Code

Run on the Mac:

```bash
cd <repo-root>
rsync -av ubuntu/reactive_nav/ turtlebot4:/home/ubuntu/reactive_nav_test/reactive_nav/
rsync -av scripts/inject_perception_event.py scripts/perception_event_io.py scripts/supervise_perception_fsm.py scripts/summarize_perception_fsm_run.py \
  turtlebot4:/home/ubuntu/reactive_nav_test/scripts/
rsync -av docs/QR_ZXING_TEST_RUN.md docs/QR_ZXING_ARCHITECTURE.md \
  turtlebot4:/home/ubuntu/reactive_nav_test/docs/
```

## 2. Capture a Small Real-Camera Dataset

Robot terminal:

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

Mac terminal:

```bash
cd <repo-root>
RUN_ID="$(date +%Y%m%d_%H%M%S)_qr_dataset"

uv run python scripts/capture_qr_dataset.py \
  --run-id "$RUN_ID" \
  --frames 10 \
  --expected-payload CHECKPOINT_TEST_1 \
  --valid \
  --bucket frontal_medium \
  --angle frontal \
  --distance medium \
  --lighting normal \
  --position center \
  --blur low \
  --partial-visibility full \
  --notes "QR centered on wall"
```

For negatives, leave the sender running and run:

```bash
cd <repo-root>
RUN_ID="$(date +%Y%m%d_%H%M%S)_qr_negative"

uv run python scripts/capture_qr_dataset.py \
  --run-id "$RUN_ID" \
  --frames 30 \
  --invalid \
  --bucket negative_no_qr \
  --angle none \
  --distance none \
  --lighting normal \
  --position none \
  --blur low \
  --partial-visibility none \
  --notes "No QR visible"
```

Datasets are saved under:

```text
output/qr_zxing_dataset/<run_id>/
```

## 3. Offline Decoder Benchmark

Run on the Mac:

```bash
cd <repo-root>
RUN_ID="$(date +%Y%m%d_%H%M%S)_qr_benchmark"
MANIFEST="$(ls -t output/qr_zxing_dataset/*/manifest.jsonl | head -n 1)"

uv run python scripts/benchmark_qr_decoders.py \
  "$MANIFEST" \
  --run-id "$RUN_ID"
```

Benchmark output:

```text
output/qr_zxing_benchmark/<run_id>/
  config.json
  per_sample.jsonl
  summary.json
  summary.csv
  summary.md
  stage_ablation.csv
  yolo_regression.json
```

The `yolo_regression.json` file is marked `not_run` by the offline script.
YOLO timing is measured in the live laptop perception log in the next phase.

## 4. Live Laptop Perception, Robot Stationary

Robot sender remains running from phase 2.

Mac terminal B:

```bash
cd <repo-root>
RUN_ID="$(date +%Y%m%d_%H%M%S)_stationary_perception"
mkdir -p "output/perception_runs/$RUN_ID"

uv run python win/yolo/recibidor.py \
  --enable-qr \
  --qr-event-path output/signals/latest_qr_event.json \
  --perception-log "output/perception_runs/$RUN_ID/laptop_perception.jsonl"
```

Mac terminal C:

```bash
cd <repo-root>
RUN_ID="$(ls -td output/perception_runs/*_stationary_perception | head -n 1 | xargs basename)"

uv run python win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --qr-source output/signals/latest_qr_event.json \
  --qr-remote-path /home/ubuntu/output/signals/latest_qr_event.json \
  --interval 0.2 \
  --log-path "output/perception_runs/$RUN_ID/sync.jsonl"
```

Check live files:

```bash
cd <repo-root>
tail -f output/perception_runs/*_stationary_perception/laptop_perception.jsonl
tail -f output/perception_runs/*_stationary_perception/sync.jsonl
```

## 5. Integrated Robot Dry-Run

Robot terminal D:

```bash
ssh turtlebot4
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

RUN_ID="$(date +%Y%m%d_%H%M%S)_qr_zxing_dryrun"
RUN_DIR="/home/ubuntu/output/fsm_perception_runs/$RUN_ID"
mkdir -p "$RUN_DIR/collision_frames"
cp /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml "$RUN_DIR/profile.yaml"

cd /home/ubuntu/reactive_nav_test
python3 -B reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p publish_zero_in_dry_run:=true \
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

Robot terminal E:

```bash
ssh turtlebot4
cd /home/ubuntu/reactive_nav_test
python3 scripts/supervise_perception_fsm.py \
  /home/ubuntu/output/fsm_perception_runs/*_qr_zxing_dryrun/reactive_nav_debug.jsonl \
  --follow
```

Expected dry-run chain:

```text
ZXing decode
-> validation_status=validated
-> sync success
-> robot qr_event_id visible
-> qr_log.jsonl gets one checkpoint record
-> intended command visible when arbiter policy allows it
-> published_linear_x=0
-> published_angular_z=0
```

## 6. Synthetic Envelope Rejection Checks

Robot terminal F:

```bash
ssh turtlebot4
cd /home/ubuntu/reactive_nav_test

python3 scripts/inject_perception_event.py qr CHECKPOINT_SYNTH \
  --semantic-qr \
  --qr-path /home/ubuntu/output/signals/latest_qr_event.json

python3 scripts/inject_perception_event.py qr CHECKPOINT_UNVALIDATED \
  --semantic-qr \
  --unvalidated-qr \
  --qr-path /home/ubuntu/output/signals/latest_qr_event.json

python3 scripts/inject_perception_event.py qr CHECKPOINT_STALE_FRAME \
  --semantic-qr \
  --source-frame-age-sec 1.0 \
  --qr-path /home/ubuntu/output/signals/latest_qr_event.json
```

The first event should be accepted once. The unvalidated and stale-source-frame
events should be rejected with explicit reasons.

## 7. Pull Evidence

Mac terminal:

```bash
cd <repo-root>
mkdir -p output/fsm_perception_runs
rsync -av turtlebot4:/home/ubuntu/output/fsm_perception_runs/ output/fsm_perception_runs/

uv run python scripts/summarize_perception_fsm_run.py output/fsm_perception_runs
```

Primary evidence:

```text
output/perception_runs/<run_id>/laptop_perception.jsonl
output/perception_runs/<run_id>/sync.jsonl
output/fsm_perception_runs/<run_id>/reactive_nav_debug.jsonl
output/fsm_perception_runs/<run_id>/qr_log.jsonl
```

## Cleanup

Stop the long-running robot/laptop processes with `Ctrl+C`.

To force a zero command on the robot:

```bash
ssh turtlebot4
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

Disable laptop QR but keep YOLO:

```bash
cd <repo-root>
uv run python win/yolo/recibidor.py
uv run python win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --interval 0.2
```

Robot-side OpenCV QR rollback:

```text
enable_qr_events=true
enable_external_qr_events=false
enable_qr_detection=true
qr_injection_path=output/qr_injection.json
```

Final decision labels:

```text
PROMOTE
ADJUST
REJECT
MEASURE
```
