# FSM, YOLO, and QR supervision run

This guide validates the perception-to-FSM chain without autonomous physical
movement.

Use it when you want to answer:

```text
Did YOLO or QR produce an event?
Was the event fresh and validated?
Did the arbiter accept or reject it?
Which FSM state did it cause?
What command was intended?
Was physical movement safely blocked?
```

The observable chain is:

```text
raw perception
  -> candidate detection
  -> validation / debounce / freshness
  -> semantic event
  -> arbiter acceptance or rejection
  -> FSM transition
  -> intended command
  -> published command gate
```

Safety state for every phase:

```text
dry_run=true
enable_motion=false
YOLO never publishes wheel commands
the behavior arbiter remains the only motion decision point
published command is zero in dry-run
```

Do not run:

```bash
python3 win/yolo/enviador.py
```

That script sends movement commands directly and is not part of this workflow.

## 0. Sync the supervision tooling to the robot

From the Mac/local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4

rsync -av ubuntu/reactive_nav/ turtlebot4:/home/ubuntu/reactive_nav_test/reactive_nav/
rsync -av scripts/inject_perception_event.py scripts/perception_event_io.py scripts/supervise_perception_fsm.py scripts/summarize_perception_fsm_run.py \
  turtlebot4:/home/ubuntu/reactive_nav_test/scripts/
rsync -av docs/FSM_PERCEPTION_SUPERVISION_RUN.md turtlebot4:/home/ubuntu/reactive_nav_test/docs/FSM_PERCEPTION_SUPERVISION_RUN.md
```

## What the structured log shows

The navigator writes JSONL records to the configured `reactive_nav_debug.jsonl`.
Each record includes a `supervision` block with fields like:

```json
{
  "supervision_schema": "perception_fsm_supervision_v1",
  "supervision": {
    "current_state": "TURNING_LEFT",
    "previous_state": "CORRIDOR_FOLLOW",
    "transition_reason": "SIGN_CONFIRMED_LEFT",
    "raw_yolo_class": "left",
    "raw_yolo_confidence": 0.99,
    "raw_yolo_age_s": 0.12,
    "yolo_confirmation_progress": "2/2",
    "yolo_event": "LEFT",
    "yolo_event_status": "accepted",
    "yolo_rejection_reason": "none",
    "raw_qr_payload": null,
    "qr_decode_status": "missing",
    "qr_confirmation_progress": "0/1",
    "qr_event": "NONE",
    "qr_event_status": "idle",
    "qr_rejection_reason": "missing",
    "active_maneuver": true,
    "maneuver_phase": "TURNING_LEFT",
    "suggested_linear_x": 0.0913,
    "suggested_angular_z": 0.0,
    "arbiter_linear_x": 0.0,
    "arbiter_angular_z": 0.45,
    "published_linear_x": 0.0,
    "published_angular_z": 0.0,
    "command_source": "active_maneuver",
    "dry_run": true,
    "enable_motion": false
  }
}
```

Interpretation:

```text
raw_yolo_*        what the detector/injector wrote
yolo_*            freshness, debounce, validation, and arbiter acceptance
raw_qr_*          QR payload before duplicate/confirmation handling
qr_*              QR confirmation/logging/duplicate status
arbiter_*         intended command after arbiter priority and safety decisions
published_*       what actually went to /cmd_vel after dry-run/motion gates
```

In this workflow, intended non-zero commands are allowed. Published non-zero
commands are not.

---

# Phase 1 — FSM only

Purpose:

```text
synthetic event -> FSM decision -> intended action
```

No camera is required. The robot must remain stationary.

## Robot terminal A — start the navigator in dry-run mode

```bash
ssh turtlebot4
```

Run:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

RUN_ID="$(date +%Y%m%d_%H%M%S)_fsm_only"
RUN_DIR="/home/ubuntu/output/fsm_perception_runs/$RUN_ID"
mkdir -p "$RUN_DIR/collision_frames"
cp /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml "$RUN_DIR/profile.yaml"

cd /home/ubuntu/reactive_nav_test
python3 -B reactive_nav/reactive_navigator.py --ros-args \
  --params-file /home/ubuntu/reactive_nav_test/reactive_nav/configs/wall_follow_less_conservative.yaml \
  -p dry_run:=true \
  -p enable_motion:=false \
  -p publish_zero_in_dry_run:=true \
  -p sign_confirm_window:=2 \
  -p sign_confirm_count:=2 \
  -p qr_confirm_count:=1 \
  -p signal_state_path:=/home/ubuntu/output/signals/latest_signal.json \
  -p qr_injection_path:=/home/ubuntu/output/qr_injection.json \
  -p qr_log_path:="$RUN_DIR/qr_log.jsonl" \
  -p persistent_log_path:="$RUN_DIR/reactive_nav_debug.jsonl" \
  -p collision_log_path:="$RUN_DIR/collision_events.jsonl" \
  -p collision_image_dir:="$RUN_DIR/collision_frames"
```

Leave it running.

Acceptance before injecting events:

```text
startup log says dry_run=True enable_motion=False
scan_count increases
published_linear_x=0
published_angular_z=0
```

## Robot terminal B — watch the supervision view

```bash
ssh turtlebot4
```

Run:

```bash
RUN_DIR="$(ls -td /home/ubuntu/output/fsm_perception_runs/*_fsm_only | head -n 1)"
cd /home/ubuntu/reactive_nav_test
python3 scripts/supervise_perception_fsm.py "$RUN_DIR/reactive_nav_debug.jsonl" --follow
```

## Robot terminal C — inject synthetic events

All commands below are safe. They write JSON files only.

### LEFT

Run twice so the configured `sign_confirm_count:=2` can confirm it:

```bash
cd /home/ubuntu/reactive_nav_test
python3 scripts/inject_perception_event.py yolo LEFT \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
sleep 0.3
python3 scripts/inject_perception_event.py yolo LEFT \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
```

Expected:

```text
raw_yolo_class=left
yolo_confirmation_progress=2/2
yolo_event=LEFT
yolo_event_status=accepted
current_state=TURNING_LEFT
command_source=active_maneuver
arbiter_angular_z > 0
published_angular_z = 0
```

### RIGHT

Wait for the previous maneuver/cooldown to clear, then:

```bash
python3 scripts/inject_perception_event.py yolo RIGHT \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
sleep 0.3
python3 scripts/inject_perception_event.py yolo RIGHT \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
```

Expected:

```text
yolo_event=RIGHT
yolo_event_status=accepted
current_state=TURNING_RIGHT
arbiter_angular_z < 0
published_angular_z = 0
```

### STOP

The current arbiter treats STOP as a U-turn maneuver:

```bash
python3 scripts/inject_perception_event.py yolo STOP \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
sleep 0.3
python3 scripts/inject_perception_event.py yolo STOP \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
```

Expected:

```text
yolo_event=STOP
yolo_event_status=accepted
current_state=TURNING_UTURN
transition_reason=STOP_SIGN_CONFIRMED_UTURN
published command remains zero
```

### QR payload

```bash
python3 scripts/inject_perception_event.py qr CHECKPOINT_TEST_1 \
  --qr-path /home/ubuntu/output/qr_injection.json
```

Expected:

```text
raw_qr_payload=CHECKPOINT_TEST_1
qr_decode_status=decoded
qr_event=CHECKPOINT_TEST_1
qr_event_status=accepted
current_state=QR_SCAN
qr_log.jsonl contains CHECKPOINT_TEST_1
published command remains zero
```

Duplicate check:

```bash
python3 scripts/inject_perception_event.py qr CHECKPOINT_TEST_1 \
  --qr-path /home/ubuntu/output/qr_injection.json
```

Expected:

```text
qr_event_status=duplicate
qr_rejection_reason=already_logged
no second checkpoint registration
```

### Negative tests

Stale YOLO:

```bash
python3 scripts/inject_perception_event.py yolo LEFT --stale \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
```

Expected:

```text
yolo_event_status=rejected
yolo_rejection_reason=stale
no YOLO-triggered turn
```

Low-confidence YOLO:

```bash
python3 scripts/inject_perception_event.py yolo LEFT --confidence 0.10 \
  --signal-path /home/ubuntu/output/signals/latest_signal.json
```

Expected:

```text
yolo_event_status=rejected
yolo_rejection_reason=low_confidence
```

Cleanup:

```bash
python3 scripts/inject_perception_event.py cleanup \
  --signal-path /home/ubuntu/output/signals/latest_signal.json \
  --qr-path /home/ubuntu/output/qr_injection.json
```

## Phase 1 summary

```bash
RUN_DIR="$(ls -td /home/ubuntu/output/fsm_perception_runs/*_fsm_only | head -n 1)"
cd /home/ubuntu/reactive_nav_test
python3 scripts/summarize_perception_fsm_run.py "$RUN_DIR"
```

Pass criteria:

```text
LEFT, RIGHT, STOP show accepted YOLO events and intended maneuvers
stale and low-confidence YOLO are rejected with explicit reasons
QR logs once and duplicate QR is identified
published command is zero throughout
```

---

# Phase 2 — Perception only

Purpose:

```text
physical sign or QR -> raw detection -> validated semantic event
```

The robot remains stationary. This phase differentiates:

```text
not detected
detected but not actionable
detected but stale
detected and validated
duplicate QR
```

## Robot terminal A — stream camera frames to laptop YOLO

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

## Laptop terminal B — run YOLO receiver

From the Mac/local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4
python3 win/yolo/recibidor.py
```

Show LEFT, RIGHT, and STOP signs to the camera one at a time.

Expected in the YOLO window/log:

```text
bounding box appears
class name matches the sign
confidence is visible
stable signal eventually writes output/signals/latest_signal.json
```

If a sign is visible but not accepted, inspect:

```text
confidence
bbox_area_ratio
bbox_center_x_ratio
actionable
stable frame count
```

## Laptop terminal C — sync YOLO latest state to robot

```bash
cd /Users/katharsis/Developer/cv/turtle4
python3 win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --interval 0.2
```

## Robot terminal D — stationary navigator/perception logger

```bash
ssh turtlebot4
```

Run:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

RUN_ID="$(date +%Y%m%d_%H%M%S)_perception_only"
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
  -p qr_injection_path:=/home/ubuntu/output/qr_injection.json \
  -p qr_log_path:="$RUN_DIR/qr_log.jsonl" \
  -p persistent_log_path:="$RUN_DIR/reactive_nav_debug.jsonl" \
  -p collision_log_path:="$RUN_DIR/collision_events.jsonl" \
  -p collision_image_dir:="$RUN_DIR/collision_frames"
```

Watch:

```bash
RUN_DIR="$(ls -td /home/ubuntu/output/fsm_perception_runs/*_perception_only | head -n 1)"
cd /home/ubuntu/reactive_nav_test
python3 scripts/supervise_perception_fsm.py "$RUN_DIR/reactive_nav_debug.jsonl" --follow
```

QR check:

```bash
tail -f "$RUN_DIR/qr_log.jsonl"
```

Phase 2 pass criteria:

```text
LEFT/RIGHT/STOP signs produce raw_yolo_class and confidence in supervision logs
accepted signs show yolo_event_status=accepted or candidates show explicit rejection/progress
QR codes produce raw_qr_payload and either accepted or duplicate status
not-detected QR shows qr_decode_status=not_detected rather than silent failure
published command remains zero
```

---

# Phase 3 — Integrated stationary dry-run

Purpose:

```text
real perception -> event -> arbiter -> FSM transition -> intended command
```

This is the full integration path with physical output blocked.

Use the same camera, YOLO receiver, and YOLO sync terminals from Phase 2. Start a
fresh navigator run:

```bash
ssh turtlebot4
```

Run:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

RUN_ID="$(date +%Y%m%d_%H%M%S)_integrated_stationary"
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
  -p qr_log_path:="$RUN_DIR/qr_log.jsonl" \
  -p persistent_log_path:="$RUN_DIR/reactive_nav_debug.jsonl" \
  -p collision_log_path:="$RUN_DIR/collision_events.jsonl" \
  -p collision_image_dir:="$RUN_DIR/collision_frames"
```

Supervision:

```bash
RUN_DIR="$(ls -td /home/ubuntu/output/fsm_perception_runs/*_integrated_stationary | head -n 1)"
cd /home/ubuntu/reactive_nav_test
python3 scripts/supervise_perception_fsm.py "$RUN_DIR/reactive_nav_debug.jsonl" --follow
```

Acceptance for each real event:

```text
LEFT:
  yolo_event=LEFT
  yolo_event_status=accepted
  current_state=TURNING_LEFT
  arbiter_angular_z > 0
  published_angular_z = 0

RIGHT:
  yolo_event=RIGHT
  yolo_event_status=accepted
  current_state=TURNING_RIGHT
  arbiter_angular_z < 0
  published_angular_z = 0

STOP:
  yolo_event=STOP
  current_state=TURNING_UTURN
  transition_reason=STOP_SIGN_CONFIRMED_UTURN
  published command remains zero

QR:
  raw_qr_payload is the expected content
  qr_event_status=accepted for first sighting or duplicate for repeat
  current_state=QR_SCAN when recently logged
  published command remains zero
```

Failure interpretation:

```text
No raw_yolo_class:
  YOLO did not detect the sign, camera stream/receiver/model issue.

raw_yolo_class present but yolo_event_status=rejected:
  Detection exists but failed freshness, confidence, area, actionability, cooldown, or LiDAR turn-clearance checks.

yolo_event_status=accepted but no turn state:
  FSM/turn-controller boundary issue.

turn state and arbiter command exist but published command non-zero:
  Dry-run/motion gate failure. Stop immediately.

raw_qr_payload missing:
  QR decoder did not see/decode the code.

qr_event_status=duplicate:
  QR worked, but content was already logged.
```

## Pull evidence back to the Mac

From the Mac/local repo:

```bash
cd /Users/katharsis/Developer/cv/turtle4
mkdir -p output/fsm_perception_runs
rsync -av turtlebot4:/home/ubuntu/output/fsm_perception_runs/ output/fsm_perception_runs/
```

Summarize locally:

```bash
RUN_DIR="$(ls -td output/fsm_perception_runs/* | head -n 1)"
.venv/bin/python scripts/summarize_perception_fsm_run.py "$RUN_DIR"
```

## Stop and cleanup

Stop navigator/camera/sync terminals with:

```text
Ctrl+C
```

Remove injected event state:

```bash
cd /home/ubuntu/reactive_nav_test
python3 scripts/inject_perception_event.py cleanup \
  --signal-path /home/ubuntu/output/signals/latest_signal.json \
  --qr-path /home/ubuntu/output/qr_injection.json
```

Optional zero command, still safe:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=2

ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

