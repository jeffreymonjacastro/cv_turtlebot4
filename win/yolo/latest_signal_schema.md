# Laptop Perception State Files

`win/yolo/recibidor.py` still writes the existing YOLO state file:

```text
output/signals/latest_signal.json
```

When started with `--enable-qr`, the same laptop process also writes a separate
validated QR event file:

```text
output/signals/latest_qr_event.json
```

The files are intentionally separate. YOLO freshness is not coupled to QR
freshness, and a QR update cannot overwrite the YOLO schema that the robot
already understands.

The reactive robot-side navigator can read this file when it is copied,
mounted, or otherwise made available on the robot.

## YOLO Required Fields

```json
{
  "direction": "left",
  "confidence": 0.95,
  "timestamp": 1710000000.0,
  "bbox_area_ratio": 0.10,
  "bbox_center_x_ratio": 0.50,
  "actionable": true
}
```

Accepted `direction` values:

```text
left
right
stop
none
```

The navigator treats the signal as stale when:

```text
now - timestamp > max_signal_age_s
```

`actionable` should only be `true` after laptop-side filtering confirms
confidence, bounding-box size, and centered placement. The robot-side arbiter
still applies its own confidence, area, freshness, debounce, cooldown, and
LiDAR safety checks. YOLO must never publish wheel commands directly.

## Current writer fields

The existing writer also includes useful diagnostics:

```json
{
  "source_frame_time": "123.456",
  "bbox_xyxy": [10, 20, 120, 180],
  "class_name": "left_arrow",
  "thresholds": {
    "action_confidence": 0.65,
    "action_min_area_ratio": 0.02,
    "action_center_x_min": 0.10,
    "action_center_x_max": 0.90,
    "stable_frames": 2
  }
}
```

The robot-side reader uses these fields only for diagnostics and duplicate
event IDs.

`win/yolo/recibidor.py` defaults these actionability gates to the
`wall_follow_less_conservative_1` profile style. Override them from the laptop
only when measuring a deliberate variant:

```bash
YOLO_ACTION_CONF_THRESHOLD=0.65
YOLO_ACTION_MIN_AREA_RATIO=0.02
YOLO_ACTION_CENTER_X_MIN=0.10
YOLO_ACTION_CENTER_X_MAX=0.90
```

## QR Semantic Event

Only validated laptop QR events are written to `latest_qr_event.json`:

```json
{
  "schema_version": "qr_semantic_event_v1",
  "event_type": "qr_checkpoint",
  "event_id": "qr:unique-id",
  "timestamp": 1710000000.0,
  "source_frame_time": "123.456000000",
  "source_received_at": 1710000000.0,
  "source_frame_age_s": 0.05,
  "qr_content": "CHECKPOINT_1",
  "raw_qr_content": "CHECKPOINT_1",
  "barcode_format": "QRCode",
  "decode_variant": "clahe_2x",
  "corners": [[10, 10], [80, 10], [80, 80], [10, 80]],
  "decode_latency_ms": 42.0,
  "validation_status": "validated",
  "confirmation_count": 2,
  "confirmation_window_s": 1.2,
  "source": "laptop_zxing"
}
```

The robot accepts the event only when:

```text
schema_version == qr_semantic_event_v1
validation_status == validated
now - timestamp <= max_qr_injection_age_s
source_frame_age_s <= max_external_qr_source_frame_age_s
event_id has not already been consumed
payload is printable, nonempty, and <= 1024 UTF-8 bytes
```

The robot logs accepted content to `qr_log.jsonl` through `QRLogger` and still
lets the arbiter decide whether QR affects the current FSM state. QR perception
never publishes wheel commands.
