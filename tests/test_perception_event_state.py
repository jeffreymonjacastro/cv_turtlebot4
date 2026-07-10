import json

import pytest

from win.yolo.perception_event_state import PerceptionEventState


def test_perception_event_state_keeps_yolo_and_qr_files_separate(tmp_path):
    state = PerceptionEventState(
        yolo_path=tmp_path / "latest_signal.json",
        qr_path=tmp_path / "latest_qr_event.json",
    )

    state.write_yolo({"direction": "left", "timestamp": 1.0})
    state.write_qr(
        {
            "schema_version": "qr_semantic_event_v1",
            "event_type": "qr_checkpoint",
            "event_id": "qr-1",
            "timestamp": 1.0,
            "source_frame_age_s": 0.1,
            "qr_content": "CHECKPOINT_1",
            "validation_status": "validated",
        }
    )

    yolo = json.loads((tmp_path / "latest_signal.json").read_text(encoding="utf-8"))
    qr = json.loads((tmp_path / "latest_qr_event.json").read_text(encoding="utf-8"))

    assert yolo["direction"] == "left"
    assert qr["qr_content"] == "CHECKPOINT_1"


def test_perception_event_state_refuses_unvalidated_qr_events(tmp_path):
    state = PerceptionEventState(
        yolo_path=tmp_path / "latest_signal.json",
        qr_path=tmp_path / "latest_qr_event.json",
    )

    with pytest.raises(ValueError):
        state.write_qr({"qr_content": "CHECKPOINT_1", "validation_status": "candidate"})
