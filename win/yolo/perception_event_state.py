#!/usr/bin/env python3
"""Thread-safe atomic state files for laptop perception semantic events."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Mapping


def atomic_write_json(path: str | Path, payload: Mapping[str, Any]) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(f"{target.stem}.{os.getpid()}.{time.monotonic_ns()}.tmp")
    tmp.write_text(json.dumps(dict(payload), indent=2, ensure_ascii=True), encoding="utf-8")
    for attempt in range(8):
        try:
            os.replace(tmp, target)
            return target
        except PermissionError:
            time.sleep(0.005 * (attempt + 1))
    # Preserve the old receiver's Windows fallback: readers already tolerate a
    # transient parse failure, while the next frame will retry atomically.
    target.write_text(tmp.read_text(encoding="utf-8"), encoding="utf-8")
    try:
        tmp.unlink()
    except OSError:
        pass
    return target


class PerceptionEventState:
    """Single process owner for separate YOLO and QR state files."""

    def __init__(self, *, yolo_path: str | Path, qr_path: str | Path):
        self.yolo_path = Path(yolo_path)
        self.qr_path = Path(qr_path)
        self._lock = threading.Lock()

    def write_yolo(self, payload: Mapping[str, Any]) -> Path:
        with self._lock:
            return atomic_write_json(self.yolo_path, payload)

    def write_qr(self, payload: Mapping[str, Any]) -> Path:
        if payload.get("validation_status") != "validated":
            raise ValueError("only validated QR events may be published")
        with self._lock:
            return atomic_write_json(self.qr_path, payload)
