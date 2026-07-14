import json

from ubuntu.reactive_nav.qr_logger import QRLogger


def test_qr_logger_confirms_and_ignores_duplicate_content(tmp_path):
    path = tmp_path / "qr_log.jsonl"
    logger = QRLogger(path, confirm_count=2)

    assert logger.observe("checkpoint-1") is None
    event = logger.observe("checkpoint-1", robot_state="QR_SCAN")
    duplicate = logger.observe("checkpoint-1", robot_state="QR_SCAN")

    assert event is not None and event.logged and not event.duplicate
    assert duplicate is not None and duplicate.duplicate and not duplicate.logged
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert len(records) == 1
    assert records[0]["qr_content"] == "checkpoint-1"


def test_qr_logger_loads_existing_seen_content(tmp_path):
    path = tmp_path / "qr_log.jsonl"
    path.write_text('{"qr_content": "already-seen"}\n', encoding="utf-8")
    logger = QRLogger(path, confirm_count=1)

    event = logger.observe("already-seen")

    assert event is not None
    assert event.duplicate
    assert not event.logged


def test_qr_logger_records_validated_external_event_once(tmp_path):
    path = tmp_path / "qr_log.jsonl"
    logger = QRLogger(path, confirm_count=2)

    event = logger.record_validated(
        " CHECKPOINT_2 ",
        source="laptop_zxing",
        event_id="qr-event-1",
        raw_content=" CHECKPOINT_2 ",
        source_frame_time="123.000000000",
        robot_state="RECOVERY",
        barcode_format="QRCode",
        decode_variant="clahe_2x",
        decode_latency_ms=12.5,
        corners=[[1, 2], [3, 4]],
    )
    duplicate = logger.record_validated("CHECKPOINT_2", source="laptop_zxing", event_id="qr-event-2")

    assert event.logged is True
    assert event.content == "CHECKPOINT_2"
    assert duplicate.duplicate is True
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert len(records) == 1
    assert records[0]["event_id"] == "qr-event-1"
    assert records[0]["source"] == "laptop_zxing"
    assert records[0]["robot_state"] == "RECOVERY"
