#!/usr/bin/env python3
"""Inject safe synthetic perception events through navigator-consumed JSON files.

This script does not publish ROS messages and never writes wheel commands.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

try:
    from scripts.perception_event_io import atomic_write_json, remove_if_exists
except ImportError:  # pragma: no cover - direct script fallback
    from perception_event_io import atomic_write_json, remove_if_exists


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SIGNAL_PATH = REPO_ROOT / "output" / "signals" / "latest_signal.json"
DEFAULT_QR_PATH = REPO_ROOT / "output" / "qr_injection.json"
DEFAULT_SEMANTIC_QR_PATH = REPO_ROOT / "output" / "signals" / "latest_qr_event.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "kind",
        choices=("yolo", "qr", "cleanup"),
        help="Event family to inject, or cleanup to remove injected state.",
    )
    parser.add_argument(
        "value",
        nargs="?",
        help="YOLO action LEFT/RIGHT/STOP/NONE or arbitrary QR payload.",
    )
    parser.add_argument("--signal-path", default=str(DEFAULT_SIGNAL_PATH), help="Path to latest_signal.json.")
    parser.add_argument("--qr-path", default=str(DEFAULT_QR_PATH), help="Path to qr_injection.json.")
    parser.add_argument("--confidence", type=float, default=0.99, help="YOLO/QR confidence value.")
    parser.add_argument("--area-ratio", type=float, default=0.20, help="YOLO bbox_area_ratio.")
    parser.add_argument("--center-x", type=float, default=0.50, help="YOLO bbox_center_x_ratio.")
    parser.add_argument("--stale", action="store_true", help="Backdate the event so the navigator rejects it as stale.")
    parser.add_argument("--age-sec", type=float, default=10.0, help="Age used with --stale.")
    parser.add_argument(
        "--non-actionable",
        action="store_true",
        help="Mark YOLO event actionable=false for rejection testing.",
    )
    parser.add_argument(
        "--source-frame-time",
        default=None,
        help="Optional source_frame_time/event identifier for YOLO.",
    )
    parser.add_argument(
        "--semantic-qr",
        action="store_true",
        help="Write the validated qr_semantic_event_v1 envelope instead of the legacy flat QR file.",
    )
    parser.add_argument(
        "--unvalidated-qr",
        action="store_true",
        help="With --semantic-qr, mark the event unvalidated for rejection testing.",
    )
    parser.add_argument(
        "--source-frame-age-sec",
        type=float,
        default=0.0,
        help="With --semantic-qr, set source_frame_age_s for freshness rejection tests.",
    )
    return parser.parse_args()


def inject_yolo(args: argparse.Namespace) -> Path:
    if not args.value:
        raise SystemExit("missing YOLO action: LEFT, RIGHT, STOP, or NONE")
    action = args.value.lower()
    aliases = {
        "left": "left",
        "l": "left",
        "right": "right",
        "r": "right",
        "stop": "stop",
        "none": "none",
    }
    if action not in aliases:
        raise SystemExit(f"unsupported YOLO action: {args.value}")
    direction = aliases[action]
    now = time.time()
    timestamp = now - max(0.0, args.age_sec) if args.stale else now
    payload = {
        "direction": direction,
        "confidence": round(float(args.confidence), 4),
        "timestamp": timestamp,
        "source_frame_time": args.source_frame_time or f"synthetic:{int(now * 1000)}",
        "bbox_xyxy": [100, 100, 300, 300] if direction != "none" else None,
        "bbox_area_ratio": round(float(args.area_ratio), 5),
        "bbox_center_x_ratio": round(float(args.center_x), 5),
        "actionable": bool(direction != "none" and not args.non_actionable),
        "class_name": direction,
        "injected": True,
        "thresholds": {
            "note": "Synthetic operator injection; navigator still applies freshness/debounce/arbiter checks."
        },
    }
    return atomic_write_json(args.signal_path, payload)


def inject_qr(args: argparse.Namespace) -> Path:
    if not args.value:
        raise SystemExit("missing QR payload")
    now = time.time()
    timestamp = now - max(0.0, args.age_sec) if args.stale else now
    if args.semantic_qr:
        target = Path(args.qr_path)
        if target == DEFAULT_QR_PATH:
            target = DEFAULT_SEMANTIC_QR_PATH
        source_frame_time = args.source_frame_time or f"synthetic:{int(now * 1000)}"
        payload = {
            "schema_version": "qr_semantic_event_v1",
            "event_type": "qr_checkpoint",
            "event_id": f"synthetic_qr:{args.value}:{timestamp:.6f}",
            "timestamp": timestamp,
            "source_frame_time": source_frame_time,
            "source_received_at": now - max(0.0, args.source_frame_age_sec),
            "source_frame_age_s": max(0.0, args.source_frame_age_sec),
            "qr_content": args.value,
            "raw_qr_content": args.value,
            "barcode_format": "QRCode",
            "decode_variant": "synthetic",
            "corners": [],
            "decode_latency_ms": 0.0,
            "validation_status": "candidate" if args.unvalidated_qr else "validated",
            "confirmation_count": 2,
            "confirmation_window_s": 1.2,
            "source": "synthetic_injector",
            "injected": True,
        }
        return atomic_write_json(target, payload)
    payload = {
        "qr_content": args.value,
        "timestamp": timestamp,
        "confidence": round(float(args.confidence), 4),
        "event_id": f"synthetic_qr:{args.value}:{timestamp:.6f}",
        "source": "synthetic_injector",
        "injected": True,
    }
    return atomic_write_json(args.qr_path, payload)


def cleanup(args: argparse.Namespace) -> None:
    removed = []
    if remove_if_exists(args.signal_path):
        removed.append(args.signal_path)
    if remove_if_exists(args.qr_path):
        removed.append(args.qr_path)
    if Path(args.qr_path) == DEFAULT_QR_PATH and remove_if_exists(DEFAULT_SEMANTIC_QR_PATH):
        removed.append(DEFAULT_SEMANTIC_QR_PATH)
    if removed:
        for path in removed:
            print(f"removed {path}")
    else:
        print("no injected files to remove")


def main() -> int:
    args = parse_args()
    if args.kind == "cleanup":
        cleanup(args)
        return 0
    if args.kind == "yolo":
        path = inject_yolo(args)
    else:
        path = inject_qr(args)
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
