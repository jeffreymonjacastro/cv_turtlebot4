#!/usr/bin/env python3
"""Shared TurtleBot UDP camera-frame transport helpers.

This module only receives/deserializes camera telemetry. It has no perception
policy and no actuator or ROS command dependency.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import socket
import time
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class ReceivedFrame:
    image: np.ndarray
    domain_id: int
    robot_name: str
    sec: int
    nanosec: int
    received_at: float
    received_monotonic: float

    @property
    def source_frame_time(self) -> str:
        return f"{self.sec}.{self.nanosec:09d}"

    @property
    def frame_id(self) -> str:
        return f"{self.robot_name}:{self.source_frame_time}"


def decode_img_parts(parts: Iterable[str], *, received_at: float | None = None) -> ReceivedFrame:
    """Decode the existing ``IMG ... base64-jpeg`` packet."""

    values = list(parts)
    if len(values) < 6 or values[0] != "IMG":
        raise ValueError("invalid IMG packet")
    domain_id = int(values[1])
    robot_name = values[2]
    sec = int(values[3])
    nanosec = int(values[4])
    jpeg_bytes = base64.b64decode(" ".join(values[5:]), validate=True)
    image = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode JPEG image")
    return ReceivedFrame(
        image=image,
        domain_id=domain_id,
        robot_name=robot_name,
        sec=sec,
        nanosec=nanosec,
        received_at=time.time() if received_at is None else float(received_at),
        received_monotonic=time.monotonic(),
    )


def do_handshake(
    sock: socket.socket,
    robot_addr: tuple[str, int],
    *,
    domain_id: int,
    pairing_code: str,
    expected_robot_name: str,
) -> None:
    """Perform the existing HELLO/ACK handshake without changing its wire shape."""

    sock.settimeout(1.0)
    while True:
        sock.sendto(f"HELLO {domain_id} {pairing_code}".encode("utf-8"), robot_addr)
        try:
            data, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        text = data.decode("utf-8", errors="ignore").strip()
        parts = text.split()
        if len(parts) < 3 or parts[0] != "ACK":
            continue
        try:
            ack_domain = int(parts[1])
        except ValueError:
            continue
        robot_name = " ".join(parts[2:])
        if ack_domain == domain_id and robot_name == expected_robot_name:
            sock.settimeout(None)
            return

