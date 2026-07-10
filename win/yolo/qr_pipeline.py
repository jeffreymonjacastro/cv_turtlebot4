#!/usr/bin/env python3
"""Latest-frame QR worker isolated from the synchronous YOLO loop."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import queue
import threading
import time
from typing import Any, Callable, Optional

import numpy as np

from win.yolo.perception_event_state import PerceptionEventState
from win.yolo.qr_validator import QRValidationResult, QRValidator
from win.yolo.qr_zxing import QRDecodeCandidate, ZXingQRDecoder


@dataclass(frozen=True)
class QRFrame:
    image: np.ndarray
    frame_id: str
    source_frame_time: str
    received_at: float
    received_monotonic: float


class LatestFrameQRWorker:
    def __init__(
        self,
        *,
        event_state: PerceptionEventState,
        decoder: Optional[ZXingQRDecoder] = None,
        validator: Optional[QRValidator] = None,
        max_hz: float = 2.0,
        log_path: str | Path | None = None,
        result_callback: Optional[Callable[[dict[str, Any]], None]] = None,
    ):
        self.event_state = event_state
        self.decoder = decoder or ZXingQRDecoder()
        self.validator = validator or QRValidator()
        self.min_interval_s = 1.0 / max(0.1, float(max_hz))
        self.log_path = Path(log_path) if log_path else None
        self.result_callback = result_callback
        self._queue: queue.Queue[QRFrame] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="qr-zxing-worker", daemon=True)
        self._last_submit = 0.0
        self._log_lock = threading.Lock()
        self.metrics = {
            "submitted": 0,
            "skipped_rate": 0,
            "replaced": 0,
            "processed": 0,
            "stale": 0,
            "failed": 0,
            "errors": 0,
            "validated": 0,
        }
        self.latest_result: dict[str, Any] = {"status": "idle"}

    def start(self) -> None:
        self._thread.start()

    def submit(self, frame: QRFrame) -> bool:
        now = time.monotonic()
        if now - self._last_submit < self.min_interval_s:
            self.metrics["skipped_rate"] += 1
            return False
        self._last_submit = now
        self.metrics["submitted"] += 1
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self.metrics["replaced"] += 1
            self._queue.put_nowait(frame)
        return True

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=max(0.0, timeout))

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._process(frame)

    def _process(self, frame: QRFrame) -> None:
        frame_age_s = max(0.0, time.monotonic() - frame.received_monotonic)
        self.metrics["processed"] += 1
        if frame_age_s > self.validator.max_frame_age_s:
            candidate = QRDecodeCandidate(None, "stale_frame")
            validation = QRValidationResult("rejected", None, "stale_frame", 0, self.validator.confirm_count)
            self.metrics["stale"] += 1
        else:
            candidate = self.decoder.decode(frame.image)
            validation = self.validator.validate(candidate, frame_id=frame.frame_id, frame_age_s=frame_age_s)
            if candidate.status in {"decoder_error", "decoder_unavailable"}:
                self.metrics["errors"] += 1
        if validation.status == "validated" and validation.normalized_payload is not None:
            event = self._event(frame, frame_age_s, candidate, validation)
            self.event_state.write_qr(event)
            self.metrics["validated"] += 1
            record = self._record(frame, frame_age_s, candidate, validation)
            record["event_id"] = event["event_id"]
            record["sent_to_state"] = True
        else:
            self.metrics["failed"] += 1
            record = self._record(frame, frame_age_s, candidate, validation)
        self.latest_result = record
        self._write_log(record)
        if self.result_callback:
            self.result_callback(dict(record))

    def _event(self, frame: QRFrame, frame_age_s: float, candidate: QRDecodeCandidate, validation: QRValidationResult) -> dict[str, Any]:
        payload = validation.normalized_payload or ""
        digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()[:12]
        return {
            "schema_version": "qr_semantic_event_v1",
            "event_type": "qr_checkpoint",
            "event_id": f"qr:{digest}:{time.time_ns()}",
            "timestamp": time.time(),
            "source_frame_time": frame.source_frame_time,
            "source_received_at": frame.received_at,
            "source_frame_age_s": round(frame_age_s, 6),
            "qr_content": payload,
            "raw_qr_content": candidate.raw_payload,
            "barcode_format": candidate.barcode_format or "QRCode",
            "decode_variant": candidate.variant,
            "corners": [list(point) for point in candidate.corners],
            "decode_latency_ms": round(candidate.decode_latency_ms, 3),
            "validation_status": "validated",
            "confirmation_count": validation.confirmation_count,
            "confirmation_window_s": self.validator.confirm_window_s,
            "source": "laptop_zxing",
        }

    def _record(self, frame: QRFrame, frame_age_s: float, candidate: QRDecodeCandidate, validation: QRValidationResult) -> dict[str, Any]:
        return {
            "timestamp": time.time(),
            "frame_id": frame.frame_id,
            "source_frame_time": frame.source_frame_time,
            "source_frame_age_s": round(frame_age_s, 6),
            "raw_qr_payload": candidate.raw_payload,
            "qr_decode_status": candidate.status,
            "qr_decode_variant": candidate.variant,
            "qr_decode_latency_ms": round(candidate.decode_latency_ms, 3),
            "qr_corners": [list(point) for point in candidate.corners],
            "qr_event": validation.normalized_payload or "NONE",
            "qr_event_status": validation.status,
            "qr_rejection_reason": validation.reason,
            "qr_confirmation_progress": f"{validation.confirmation_count}/{validation.confirmation_required}",
            "qr_cooldown_remaining_s": round(validation.cooldown_remaining_s, 3),
            "last_accepted_qr": self.validator.last_accepted_payload,
            "sent_to_state": False,
            "queue_metrics": dict(self.metrics),
        }

    def _write_log(self, record: dict[str, Any]) -> None:
        if self.log_path is None:
            return
        with self._log_lock:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=True) + "\n")
