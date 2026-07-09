import json
import time

from ubuntu.reactive_nav.reactive_navigator import read_signal_state


def test_signal_state_uses_file_mtime_when_payload_timestamp_is_stale(tmp_path):
    path = tmp_path / "latest_signal.json"
    path.write_text(
        json.dumps(
            {
                "direction": "stop",
                "confidence": 0.90,
                "bbox_area_ratio": 0.05,
                "bbox_center_x_ratio": 0.50,
                "timestamp": time.time() - 1000.0,
            }
        ),
        encoding="utf-8",
    )

    signal = read_signal_state(
        path,
        max_age_s=1.5,
        min_confidence=0.55,
        min_area_ratio=0.015,
        center_min=0.10,
        center_max=0.90,
    )

    assert signal.direction == "stop"
    assert signal.actionable is True
    assert signal.stale is False
    assert signal.reason == "fresh"
