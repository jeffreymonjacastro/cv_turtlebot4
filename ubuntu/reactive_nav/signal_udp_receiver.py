#!/usr/bin/env python3
"""Receive YOLO latest-signal UDP packets and write the robot-side JSON file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time


DEFAULT_OUTPUT = Path.home() / "output" / "signals" / "latest_signal.json"
DEFAULT_PORT = 6611
MAX_PACKET_BYTES = 60_000


def parse_signal_packet(data: bytes) -> dict:
    if len(data) > MAX_PACKET_BYTES:
        raise ValueError(f"packet too large: {len(data)} bytes")
    payload = json.loads(data.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("packet must contain a JSON object")
    return payload


def write_signal_state(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=True, separators=(",", ":")),
        encoding="utf-8",
    )
    tmp_path.replace(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-ip", default="0.0.0.0", help="Local IP to bind.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to listen on.")
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="Path read by reactive_navigator.py signal_state_path.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = Path(args.output).expanduser()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_ip, int(args.port)))

    print(f"[SIGNAL-UDP] listening on {args.listen_ip}:{args.port}")
    print(f"[SIGNAL-UDP] writing {output}")

    try:
        while True:
            data, addr = sock.recvfrom(MAX_PACKET_BYTES + 1)
            try:
                payload = parse_signal_packet(data)
                write_signal_state(output, payload)
            except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                print(f"[SIGNAL-UDP] WARN: ignored packet from {addr}: {exc}")
                continue

            direction = payload.get("direction", "unknown")
            timestamp = float(payload.get("timestamp") or 0.0)
            age = time.time() - timestamp if timestamp > 0.0 else float("inf")
            print(f"[SIGNAL-UDP] wrote direction={direction} age={age:.2f}s from={addr[0]}")
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
