#!/usr/bin/env python3
"""Shared JSON-file helpers for safe perception event tooling."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Mapping


def atomic_write_json(path: str | Path, payload: Mapping[str, Any]) -> Path:
    """Write JSON using same-directory replacement so readers never see partial JSON."""

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(f"{target.stem}.{os.getpid()}.{target.suffix or '.json'}.tmp")
    tmp.write_text(json.dumps(dict(payload), indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(target)
    return target


def remove_if_exists(path: str | Path) -> bool:
    target = Path(path)
    try:
        target.unlink()
        return True
    except FileNotFoundError:
        return False
