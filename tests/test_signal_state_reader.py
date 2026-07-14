import json
import time

from ubuntu.reactive_nav.reactive_navigator import read_injected_qr_event, read_signal_state


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


def test_signal_state_robot_profile_can_relax_laptop_actionable_flag(tmp_path):
    path = tmp_path / "latest_signal.json"
    path.write_text(
        json.dumps(
            {
                "direction": "left",
                "confidence": 0.42,
                "bbox_area_ratio": 0.015,
                "bbox_center_x_ratio": 0.05,
                "actionable": False,
                "timestamp": time.time(),
            }
        ),
        encoding="utf-8",
    )

    signal = read_signal_state(
        path,
        max_age_s=3.0,
        min_confidence=0.30,
        min_area_ratio=0.01,
        center_min=0.03,
        center_max=0.97,
    )

    assert signal.direction == "left"
    assert signal.actionable is True
    assert signal.stale is False


def test_validated_qr_envelope_is_fresh_and_normalized(tmp_path):
    path = tmp_path / "latest_qr_event.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "qr_semantic_event_v1",
                "event_type": "qr_checkpoint",
                "event_id": "qr-1",
                "timestamp": time.time(),
                "source_frame_time": "123.000000000",
                "source_frame_age_s": 0.1,
                "qr_content": " CHECKPOINT_1 ",
                "raw_qr_content": " CHECKPOINT_1 ",
                "barcode_format": "QRCode",
                "decode_variant": "clahe_2x",
                "decode_latency_ms": 14.0,
                "validation_status": "validated",
                "source": "laptop_zxing",
            }
        ),
        encoding="utf-8",
    )

    event = read_injected_qr_event(path, max_age_s=2.0, max_source_frame_age_s=0.5)

    assert event["fresh"] is True
    assert event["qr_event"] == "CHECKPOINT_1"
    assert event["event_id"] == "qr-1"
    assert event["is_semantic_event"] is True
    assert event["decode_variant"] == "clahe_2x"


def test_qr_envelope_rejects_unvalidated_stale_and_invalid_payload(tmp_path):
    base = {
        "schema_version": "qr_semantic_event_v1",
        "event_type": "qr_checkpoint",
        "event_id": "qr-1",
        "timestamp": time.time(),
        "source_frame_age_s": 0.1,
        "qr_content": "CHECKPOINT_1",
        "validation_status": "validated",
    }
    cases = [
        ({"validation_status": "candidate"}, "event_not_validated"),
        ({"source_frame_age_s": None}, "missing_source_frame_age"),
        ({"source_frame_age_s": 0.6}, "stale_source_frame:0.60s"),
        ({"qr_content": "bad\npayload"}, "invalid_payload"),
        ({"timestamp": time.time() - 5.0}, "stale:"),
    ]
    for index, (override, reason_prefix) in enumerate(cases):
        path = tmp_path / f"latest_qr_event_{index}.json"
        payload = dict(base)
        payload.update(override)
        path.write_text(json.dumps(payload), encoding="utf-8")

        event = read_injected_qr_event(path, max_age_s=2.0, max_source_frame_age_s=0.5)

        assert event["fresh"] is False
        assert event["qr_event_status"] == "rejected"
        assert event["qr_rejection_reason"].startswith(reason_prefix)
