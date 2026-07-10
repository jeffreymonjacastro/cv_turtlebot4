#!/usr/bin/env python3
"""Persistent QR checkpoint evidence logging."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Deque, Iterable, Optional, Set
import unicodedata


@dataclass(frozen=True)
class QREvent:
    content: str
    logged: bool
    duplicate: bool
    path: str
    status: str = "logged"
    reason: str = "none"


class QRLogger:
    def __init__(
        self,
        log_path: str | Path = "output/qr_log.jsonl",
        confirm_count: int = 2,
        window_size: int = 5,
    ):
        self.log_path = Path(log_path)
        self.confirm_count = max(1, int(confirm_count))
        self.window_size = max(self.confirm_count, int(window_size))
        self._recent: Deque[str] = deque(maxlen=self.window_size)
        self._seen: Set[str] = set()
        self._load_seen()

    def _load_seen(self) -> None:
        if not self.log_path.exists():
            return
        try:
            for line in self.log_path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                payload = json.loads(line)
                content = payload.get("qr_content")
                if content:
                    self._seen.add(str(content))
        except (OSError, json.JSONDecodeError):
            return

    def observe(
        self,
        content: Optional[str],
        *,
        source: str = "camera",
        frame_id: Optional[str] = None,
        robot_state: Optional[str] = None,
        confidence: Optional[float] = None,
        context: Optional[dict] = None,
    ) -> Optional[QREvent]:
        content = self.normalize_content(content)
        if content is None:
            return None

        self._recent.append(content)
        if self._recent_count(content) < self.confirm_count:
            return None

        duplicate = content in self._seen
        if duplicate:
            return QREvent(content, False, True, str(self.log_path), status="duplicate", reason="already_logged")

        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "qr_content": content,
            "source": source,
            "frame_id": frame_id,
            "robot_state": robot_state,
            "confidence": confidence,
        }
        if context:
            record["context"] = context
        self._write_record(content, record)
        return QREvent(content, True, False, str(self.log_path), status="logged", reason="confirmed")

    def record_validated(
        self,
        content: Optional[str],
        *,
        source: str,
        event_id: str,
        raw_content: Optional[str] = None,
        source_frame_time: Optional[str] = None,
        robot_state: Optional[str] = None,
        barcode_format: Optional[str] = None,
        decode_variant: Optional[str] = None,
        decode_latency_ms: Optional[float] = None,
        corners: Optional[list] = None,
        context: Optional[dict] = None,
    ) -> QREvent:
        """Persist one already validated semantic event without frame re-confirmation."""

        normalized = self.normalize_content(content)
        if normalized is None:
            return QREvent(str(content or ""), False, False, str(self.log_path), status="rejected", reason="invalid_payload")
        if normalized in self._seen:
            return QREvent(normalized, False, True, str(self.log_path), status="duplicate", reason="already_logged")
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "qr_content": normalized,
            "raw_qr_content": raw_content,
            "source": source,
            "event_id": event_id,
            "frame_id": source_frame_time,
            "source_frame_time": source_frame_time,
            "robot_state": robot_state,
            "confidence": None,
            "barcode_format": barcode_format,
            "decode_variant": decode_variant,
            "decode_latency_ms": decode_latency_ms,
            "corners": corners or [],
        }
        if context:
            record["context"] = context
        self._write_record(normalized, record)
        return QREvent(normalized, True, False, str(self.log_path), status="logged", reason="validated_external_event")

    @staticmethod
    def normalize_content(content: Optional[str], *, max_bytes: int = 1024) -> Optional[str]:
        if content is None:
            return None
        normalized = unicodedata.normalize("NFC", str(content)).strip()
        if not normalized or not normalized.isprintable():
            return None
        if len(normalized.encode("utf-8")) > max_bytes:
            return None
        return normalized

    def _write_record(self, content: str, record: dict) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=True) + "\n")
        self._seen.add(content)

    def _recent_count(self, content: str) -> int:
        return sum(1 for item in self._recent if item == content)

    def confirmation_count(self, content: Optional[str]) -> int:
        if not content:
            return 0
        return self._recent_count(str(content).strip())

    def confirmation_progress(self, content: Optional[str]) -> str:
        return f"{min(self.confirmation_count(content), self.confirm_count)}/{self.confirm_count}"

    @property
    def seen(self) -> Iterable[str]:
        return tuple(sorted(self._seen))
