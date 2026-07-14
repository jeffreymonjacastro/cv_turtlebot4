import json
import time

import cv2
import numpy as np

from win.yolo.perception_event_state import PerceptionEventState
from win.yolo.qr_pipeline import LatestFrameQRWorker, QRFrame
from win.yolo.qr_validator import QRValidator, normalize_qr_payload
from win.yolo.qr_zxing import QRDecodeCandidate, ZXingQRDecoder


def _qr_image(payload="milestone_gamma"):
    encoder = cv2.QRCodeEncoder_create()
    encoded = encoder.encode(payload)
    return cv2.cvtColor(encoded, cv2.COLOR_GRAY2BGR)


def test_zxing_decodes_fixture_qr_payload():
    decoder = ZXingQRDecoder(("original", "gray_2x", "clahe_2x"))
    result = decoder.decode(_qr_image())

    assert result.status == "decoded_candidate"
    assert result.raw_payload == "milestone_gamma"
    assert result.barcode_format is not None and "QR" in result.barcode_format.upper()


def test_validator_normalizes_confirms_cools_down_and_resets():
    validator = QRValidator(confirm_count=2, confirm_window_s=1.2, duplicate_cooldown_s=30.0)
    candidate = QRDecodeCandidate("  cafe\u0301 checkpoint  ", "decoded_candidate", barcode_format="QRCode")

    first = validator.validate(candidate, frame_id="frame-1", frame_age_s=0.1, now=10.0)
    same_frame = validator.validate(candidate, frame_id="frame-1", frame_age_s=0.1, now=10.1)
    second = validator.validate(candidate, frame_id="frame-2", frame_age_s=0.1, now=10.2)
    duplicate = validator.validate(candidate, frame_id="frame-3", frame_age_s=0.1, now=10.3)

    assert first.status == "candidate"
    assert same_frame.confirmation_count == 1
    assert second.status == "validated"
    assert second.normalized_payload == "café checkpoint"
    assert duplicate.status == "rejected"
    assert duplicate.reason == "duplicate_cooldown"

    validator.reset()
    after_reset = validator.validate(candidate, frame_id="frame-4", frame_age_s=0.1, now=50.0)
    assert after_reset.status == "candidate"


def test_validator_rejects_stale_and_control_payloads():
    validator = QRValidator(max_frame_age_s=0.5)

    stale = validator.validate(
        QRDecodeCandidate("ok", "decoded_candidate", barcode_format="QRCode"),
        frame_id="old-frame",
        frame_age_s=0.6,
    )
    control, reason = normalize_qr_payload("bad\npayload")

    assert stale.status == "rejected"
    assert stale.reason == "stale_frame"
    assert control is None
    assert reason == "control_characters"


class _FakeDecoder:
    def __init__(self, payload="CHECKPOINT_1"):
        self.payload = payload
        self.calls = 0

    def decode(self, _image):
        self.calls += 1
        return QRDecodeCandidate(self.payload, "decoded_candidate", barcode_format="QRCode", variant="fake")


def test_latest_frame_worker_rejects_stale_before_decoding_and_writes_validated_event(tmp_path):
    event_state = PerceptionEventState(
        yolo_path=tmp_path / "latest_signal.json",
        qr_path=tmp_path / "latest_qr_event.json",
    )
    fake_decoder = _FakeDecoder()
    validator = QRValidator(confirm_count=1, max_frame_age_s=0.5)
    log_path = tmp_path / "perception.jsonl"
    worker = LatestFrameQRWorker(
        event_state=event_state,
        decoder=fake_decoder,
        validator=validator,
        log_path=log_path,
    )
    image = np.zeros((10, 10, 3), dtype=np.uint8)

    worker._process(
        QRFrame(
            image=image,
            frame_id="stale",
            source_frame_time="1.000000000",
            received_at=time.time() - 1.0,
            received_monotonic=time.monotonic() - 1.0,
        )
    )
    assert fake_decoder.calls == 0
    assert worker.latest_result["qr_rejection_reason"] == "stale_frame"

    worker._process(
        QRFrame(
            image=image,
            frame_id="fresh",
            source_frame_time="2.000000000",
            received_at=time.time(),
            received_monotonic=time.monotonic(),
        )
    )
    event = json.loads((tmp_path / "latest_qr_event.json").read_text(encoding="utf-8"))
    logs = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]

    assert fake_decoder.calls == 1
    assert event["schema_version"] == "qr_semantic_event_v1"
    assert event["qr_content"] == "CHECKPOINT_1"
    assert event["validation_status"] == "validated"
    assert logs[-1]["sent_to_state"] is True
