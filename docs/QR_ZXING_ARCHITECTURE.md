# Laptop ZXing QR Architecture

## Before

```text
/oakd/rgb/preview/image_raw
  -> debug_image_udp_sender.py
  -> win/yolo/recibidor.py
  -> YOLO latest_signal.json
  -> enviador_yolo.py
  -> robot reactive_navigator.py

/oakd/rgb/preview/image_raw
  -> robot reactive_navigator.py
  -> synchronous OpenCV QR decode
  -> QRLogger / qr_log.jsonl
```

QR and YOLO did not decode from the same laptop frame. QR work happened inside
the robot image callback, so slow decoding could delay ROS callbacks.

## After

```text
/oakd/rgb/preview/image_raw
  -> debug_image_udp_sender.py
  -> UDP JPEG frame
  -> win/yolo/recibidor.py
       -> existing YOLO path
       -> output/signals/latest_signal.json
       -> isolated ZXing QR worker
       -> output/signals/latest_qr_event.json
  -> win/reactive_nav/enviador_yolo.py
       -> atomic YOLO sync
       -> optional atomic QR sync
  -> robot reactive_navigator.py
       -> LiDAR/emergency
       -> active maneuver
       -> QR_SCAN hold/checkpoint logging
       -> YOLO sign handling
       -> navigation
```

The laptop QR path is semantic-only. It does not import ROS command messages,
does not publish `/cmd_vel`, and does not choose navigation commands.

## State Files

YOLO remains unchanged:

```text
output/signals/latest_signal.json
/home/ubuntu/output/signals/latest_signal.json
```

Validated QR events use a separate file:

```text
output/signals/latest_qr_event.json
/home/ubuntu/output/signals/latest_qr_event.json
```

QR envelope:

```json
{
  "schema_version": "qr_semantic_event_v1",
  "event_type": "qr_checkpoint",
  "event_id": "qr:unique",
  "timestamp": 1710000000.0,
  "source_frame_time": "123.456000000",
  "source_received_at": 1710000000.0,
  "source_frame_age_s": 0.05,
  "qr_content": "CHECKPOINT_1",
  "raw_qr_content": "CHECKPOINT_1",
  "barcode_format": "QRCode",
  "decode_variant": "clahe_2x",
  "corners": [],
  "decode_latency_ms": 40.0,
  "validation_status": "validated",
  "confirmation_count": 2,
  "confirmation_window_s": 1.2,
  "source": "laptop_zxing"
}
```

## Laptop Decoder

`win/yolo/qr_zxing.py` runs a QR-only early-exit cascade:

```text
original
gray
CLAHE
2x gray
2x CLAHE
mild sharpen
center_80 crop
inverted gray
```

Rotations are disabled. A stage should stay in the promoted default only when
the real dataset shows that it adds true decodes, adds no false positives, and
keeps latency within budget.

## Validation

`win/yolo/qr_validator.py` accepts payloads that are:

```text
Unicode NFC normalized
outer whitespace trimmed
case and internal whitespace preserved
nonempty printable text
<= 1024 UTF-8 bytes
QRCode format only
```

Default confirmation:

```text
2 observations of the same normalized payload
distinct frame IDs
within 1.2 seconds
same-payload resend cooldown of 30 seconds
stale source frames rejected after 0.5 seconds
```

## Scheduling

`win/yolo/qr_pipeline.py` uses one worker thread and a queue of size one.
Newer frames replace older queued frames. The worker records submitted,
rate-skipped, replaced, stale, failed, decoder-error, processed, and validated
counts. Work older than the configured freshness budget is rejected before
ZXing runs.

If ZXing import or decoding fails, the QR worker records the error and YOLO
continues.

## Robot Switches

Promoted laptop-QR profiles use:

```text
enable_qr_events=true
enable_external_qr_events=true
enable_qr_detection=false
qr_injection_path=/home/ubuntu/output/signals/latest_qr_event.json
max_external_qr_source_frame_age_s=0.5
```

`enable_qr_events` is the master switch. `enable_external_qr_events` enables
validated laptop envelopes. `enable_qr_detection` controls the old robot-side
OpenCV fallback.

## Observability

Evidence locations:

```text
output/perception_runs/<run_id>/laptop_perception.jsonl
output/perception_runs/<run_id>/sync.jsonl
/home/ubuntu/output/autonomous_runs/<run_id>/reactive_nav_debug.jsonl
/home/ubuntu/output/autonomous_runs/<run_id>/qr_log.jsonl
```

Robot debug records include QR event ID, schema, validation status, source
frame age, decode variant, decode latency, FSM state/reason, requested command,
published command, and motion flags.

## Budgets and Gates

Promotion requires:

```text
zero validated events on negative/malformed samples
duplicates produce one semantic send and one robot checkpoint record
validated recall no worse than the existing OpenCV cascade
at least one weak practical bucket improves by 15 percentage points
no bucket regresses by more than 5 percentage points
QR median/p95 decode latency <= 150/250 ms
accepted-event end-to-end p95 latency <= 1.0 s
YOLO schema unchanged
YOLO FPS >= 90% of baseline
YOLO p95 latency <= 110% baseline or +20 ms
no queue backlog; queue depth remains one
```

Offline and stationary success are not physical movement validation.

## Rollback

Disable laptop QR and keep YOLO:

```bash
uv run python win/yolo/recibidor.py
uv run python win/reactive_nav/enviador_yolo.py \
  --robot turtlebot4 \
  --source output/signals/latest_signal.json \
  --remote-path /home/ubuntu/output/signals/latest_signal.json \
  --interval 0.2
```

Restore the old robot OpenCV QR fallback:

```text
enable_qr_events=true
enable_external_qr_events=false
enable_qr_detection=true
qr_injection_path=output/qr_injection.json
```

Immediate QR-off mode:

```text
enable_qr_events=false
```
