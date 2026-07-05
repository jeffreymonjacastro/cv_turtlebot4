#!/usr/bin/env python3
"""Persistent QR checkpoint evidence logging."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Deque, Iterable, Optional, Set


@dataclass(frozen=True)
class QREvent:
    content: str
    logged: bool
    duplicate: bool
    path: str


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
        if not content:
            return None
        content = str(content).strip()
        if not content:
            return None

        self._recent.append(content)
        if self._recent_count(content) < self.confirm_count:
            return None

        duplicate = content in self._seen
        if duplicate:
            return QREvent(content, False, True, str(self.log_path))

        self.log_path.parent.mkdir(parents=True, exist_ok=True)
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
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=True) + "\n")
        self._seen.add(content)
        return QREvent(content, True, False, str(self.log_path))

    def _recent_count(self, content: str) -> int:
        return sum(1 for item in self._recent if item == content)

    @property
    def seen(self) -> Iterable[str]:
        return tuple(sorted(self._seen))

