#!/usr/bin/env python3
"""Capture labeled QR benchmark frames from the existing robot UDP camera stream.

This script only receives camera telemetry and writes image/manifest evidence.
It never publishes ROS messages and never sends wheel commands.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import socket
import sys
import time

import cv2

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from win.yolo.frame_stream import decode_img_parts, do_handshake


DEFAULT_ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")
DEFAULT_ROBOT_PORT = 6610
DESIRED_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "2"))
PAIRING_CODE = os.environ.get("PAIRING_CODE", "ROBOT_PAIRING_CODE")
EXPECTED_ROBOT_NAME = os.environ.get("ROBOT_NAME", "turtlebot4")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default=DEFAULT_ROBOT_IP, help="Robot IP for the UDP camera stream.")
    parser.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT_PORT)
    parser.add_argument("--out-dir", default=str(REPO_ROOT / "output" / "qr_zxing_dataset"))
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--frames", type=int, default=10, help="Number of IMG frames to save.")
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--expected-payload", default="", help="Expected QR text for positive samples.")
    valid_group = parser.add_mutually_exclusive_group()
    valid_group.add_argument("--valid", dest="valid", action="store_true", default=True)
    valid_group.add_argument("--invalid", dest="valid", action="store_false")
    parser.add_argument("--bucket", default="unspecified", help="Scenario bucket, e.g. frontal_medium or negative.")
    parser.add_argument("--angle", default="unspecified")
    parser.add_argument("--distance", default="unspecified")
    parser.add_argument("--lighting", default="unspecified")
    parser.add_argument("--position", default="unspecified")
    parser.add_argument("--blur", default="unspecified")
    parser.add_argument("--partial-visibility", default="unspecified")
    parser.add_argument("--notes", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.out_dir).expanduser().resolve() / run_id
    frames_dir = run_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = run_dir / "manifest.jsonl"
    config_path = run_dir / "capture_config.json"

    config = vars(args).copy()
    config["run_id"] = run_id
    config["run_dir"] = str(run_dir)
    config_path.write_text(json.dumps(config, indent=2, ensure_ascii=True), encoding="utf-8")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (args.robot_ip, args.robot_port)
    print(f"[CAPTURE] output={run_dir}")
    print(f"[CAPTURE] handshaking with {robot_addr}")
    do_handshake(
        sock,
        robot_addr,
        domain_id=DESIRED_DOMAIN_ID,
        pairing_code=PAIRING_CODE,
        expected_robot_name=EXPECTED_ROBOT_NAME,
    )
    print("[CAPTURE] receiving IMG packets; Ctrl+C to stop early")

    saved = 0
    started = time.monotonic()
    try:
        with manifest_path.open("a", encoding="utf-8") as manifest:
            while saved < max(1, args.frames):
                if time.monotonic() - started > max(0.1, args.max_seconds):
                    break
                data, _addr = sock.recvfrom(65535)
                parts = data.decode("utf-8", errors="ignore").split()
                if not parts or parts[0] != "IMG":
                    continue
                received_at = time.time()
                frame = decode_img_parts(parts, received_at=received_at)
                saved += 1
                filename = f"frame_{saved:06d}.jpg"
                image_path = frames_dir / filename
                cv2.imwrite(str(image_path), frame.image)
                record = {
                    "path": str(image_path),
                    "relative_path": str(image_path.relative_to(run_dir)),
                    "expected_payload": args.expected_payload,
                    "valid": bool(args.valid),
                    "bucket": args.bucket,
                    "angle": args.angle,
                    "distance": args.distance,
                    "lighting": args.lighting,
                    "position": args.position,
                    "blur": args.blur,
                    "partial_visibility": args.partial_visibility,
                    "ros_timestamp": frame.source_frame_time,
                    "capture_time": received_at,
                    "operator_notes": args.notes,
                    "robot_name": frame.robot_name,
                    "domain_id": frame.domain_id,
                }
                manifest.write(json.dumps(record, ensure_ascii=True) + "\n")
                manifest.flush()
                print(f"[CAPTURE] saved {saved}/{args.frames}: {image_path}")
    except KeyboardInterrupt:
        print("\n[CAPTURE] stopped by operator")
    finally:
        sock.close()

    print(f"[CAPTURE] saved={saved} manifest={manifest_path}")
    return 0 if saved > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
