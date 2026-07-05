# Latest YOLO Signal State

`win/yolo/recibidor.py` writes:

```text
output/signals/latest_signal.json
```

The reactive robot-side navigator can read this file when it is copied,
mounted, or otherwise made available on the robot.

## Required fields

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
    "action_confidence": 0.9,
    "action_min_area_ratio": 0.18,
    "action_center_x_min": 0.25,
    "action_center_x_max": 0.75,
    "stable_frames": 2
  }
}
```

The robot-side reader uses these fields only for diagnostics and duplicate
event IDs.

