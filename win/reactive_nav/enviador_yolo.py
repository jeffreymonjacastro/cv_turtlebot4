#!/usr/bin/env python3
"""Send YOLO latest-signal state from the laptop to the TurtleBot over UDP."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = REPO_ROOT / "output" / "signals" / "latest_signal.json"
DEFAULT_ROBOT_IP = os.environ.get("ROBOT_IP", "10.60.199.200")
DEFAULT_PORT = int(os.environ.get("YOLO_SIGNAL_PORT", "6611"))
MAX_PACKET_BYTES = 60_000


def build_signal_packet(source: Path) -> tuple[bytes, dict]:
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("latest_signal.json must contain a JSON object")
    packet = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
    if len(packet) > MAX_PACKET_BYTES:
        raise ValueError(f"latest_signal.json is too large for one UDP packet: {len(packet)} bytes")
    return packet, payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default=str(DEFAULT_SOURCE),
        help="Local latest_signal.json written by win/yolo/recibidor.py.",
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help="TurtleBot IP address.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help="UDP port listened to by ubuntu/reactive_nav/signal_udp_receiver.py.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Polling interval in seconds.",
    )
    parser.add_argument(
        "--send-every-interval",
        action="store_true",
        help="Send every interval instead of only when the source mtime changes.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress successful send messages.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = Path(args.source).expanduser().resolve()
    interval = max(0.05, float(args.interval))
    target = (args.robot_ip, int(args.port))

    print(f"[YOLO-UDP] source={source}")
    print(f"[YOLO-UDP] destination={target[0]}:{target[1]}")
    print(f"[YOLO-UDP] interval={interval:.2f}s")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_mtime_ns = None
    last_missing_log = 0.0
    try:
        while True:
            try:
                stat = source.stat()
            except FileNotFoundError:
                now = time.monotonic()
                if now - last_missing_log >= 2.0:
                    print(f"[YOLO-UDP] waiting for {source}")
                    last_missing_log = now
                time.sleep(interval)
                continue

            changed = stat.st_mtime_ns != last_mtime_ns
            if changed or args.send_every_interval:
                try:
                    packet, payload = build_signal_packet(source)
                except (OSError, json.JSONDecodeError, ValueError) as exc:
                    print(f"[YOLO-UDP] WARN: could not read signal: {exc}", file=sys.stderr)
                else:
                    sock.sendto(packet, target)
                    last_mtime_ns = stat.st_mtime_ns
                    if not args.quiet:
                        age = time.time() - stat.st_mtime
                        direction = payload.get("direction", "unknown")
                        print(
                            f"[YOLO-UDP] sent latest_signal.json "
                            f"direction={direction} age={age:.2f}s bytes={len(packet)}"
                        )

            time.sleep(interval)
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
