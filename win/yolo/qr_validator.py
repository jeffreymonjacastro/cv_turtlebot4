#!/usr/bin/env python3
"""QR payload normalization, temporal confirmation, and duplicate policy."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import time
import unicodedata
from typing import Deque, Dict, Optional

from win.yolo.qr_zxing import QRDecodeCandidate


@dataclass(frozen=True)
class QRValidationResult:
    status: str
    normalized_payload: Optional[str]
    reason: str
    confirmation_count: int
    confirmation_required: int
    cooldown_remaining_s: float = 0.0


def normalize_qr_payload(raw: Optional[str], *, max_bytes: int = 1024) -> tuple[Optional[str], str]:
    if raw is None:
        return None, "empty_payload"
    normalized = unicodedata.normalize("NFC", str(raw)).strip()
    if not normalized:
        return None, "empty_payload"
    if not normalized.isprintable():
        return None, "control_characters"
    if len(normalized.encode("utf-8")) > max_bytes:
        return None, "payload_too_large"
    return normalized, "none"


class QRValidator:
    def __init__(
        self,
        *,
        confirm_count: int = 2,
        confirm_window_s: float = 1.2,
        duplicate_cooldown_s: float = 30.0,
        max_frame_age_s: float = 0.5,
        enabled: bool = True,
    ):
        self.confirm_count = max(1, int(confirm_count))
        self.confirm_window_s = max(0.05, float(confirm_window_s))
        self.duplicate_cooldown_s = max(0.0, float(duplicate_cooldown_s))
        self.max_frame_age_s = max(0.0, float(max_frame_age_s))
        self.enabled = bool(enabled)
        self._observations: Dict[str, Deque[tuple[str, float]]] = defaultdict(deque)
        self._accepted_at: Dict[str, float] = {}
        self.last_accepted_payload: Optional[str] = None

    def reset(self) -> None:
        self._observations.clear()
        self._accepted_at.clear()
        self.last_accepted_payload = None

    def validate(
        self,
        candidate: QRDecodeCandidate,
        *,
        frame_id: str,
        frame_age_s: float,
        now: Optional[float] = None,
    ) -> QRValidationResult:
        now = time.monotonic() if now is None else float(now)
        if not self.enabled:
            return QRValidationResult("rejected", None, "qr_disabled", 0, self.confirm_count)
        if frame_age_s > self.max_frame_age_s:
            return QRValidationResult("rejected", None, "stale_frame", 0, self.confirm_count)
        if candidate.status != "decoded_candidate":
            return QRValidationResult(candidate.status, None, candidate.error or candidate.status, 0, self.confirm_count)
        normalized, reason = normalize_qr_payload(candidate.raw_payload)
        if normalized is None:
            return QRValidationResult("rejected", None, reason, 0, self.confirm_count)
        if candidate.barcode_format and "QR" not in candidate.barcode_format.upper():
            return QRValidationResult("rejected", normalized, "unsupported_format", 0, self.confirm_count)

        last_accepted = self._accepted_at.get(normalized)
        if last_accepted is not None:
            remaining = self.duplicate_cooldown_s - (now - last_accepted)
            if remaining > 0.0:
                return QRValidationResult("rejected", normalized, "duplicate_cooldown", 0, self.confirm_count, remaining)

        observations = self._observations[normalized]
        cutoff = now - self.confirm_window_s
        while observations and observations[0][1] < cutoff:
            observations.popleft()
        if not any(item[0] == frame_id for item in observations):
            observations.append((frame_id, now))
        count = len({item[0] for item in observations})
        if count < self.confirm_count:
            return QRValidationResult("candidate", normalized, "awaiting_confirmation", count, self.confirm_count)

        self._accepted_at[normalized] = now
        self.last_accepted_payload = normalized
        observations.clear()
        return QRValidationResult("validated", normalized, "none", count, self.confirm_count)

