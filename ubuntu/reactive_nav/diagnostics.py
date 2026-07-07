#!/usr/bin/env python3
"""Console and UDP diagnostics for the reactive navigation node."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any, Dict, Optional, Tuple


@dataclass(frozen=True)
class DiagnosticSnapshot:
    state: str
    front: float
    left: float
    right: float
    rear: float
    nearest_dist: float
    nearest_angle: float
    gap_start: float
    gap_end: float
    turn_hint: float
    speed: float
    yaw: float


class UdpDiagnostics:
    def __init__(
        self,
        logger,
        *,
        enabled: bool = True,
        port: int = 6612,
        robot_name: str = "turtlebot4_rensso_mora",
        pairing_code: str = "ROBOT_A_2",
    ):
        self.logger = logger
        self.enabled = enabled
        self.port = int(port)
        self.robot_name = robot_name
        self.pairing_code = pairing_code
        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "2"))
        self.authorized_addr: Optional[Tuple[str, int]] = None
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

        if self.enabled:
            self._start()

    def _start(self) -> None:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("0.0.0.0", self.port))
            self._sock.settimeout(0.5)
        except OSError as exc:
            self.enabled = False
            self.logger.warn(f"[DIAG] UDP diagnostics disabled; bind failed on {self.port}: {exc}")
            return
        self._running = True
        self._thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()
        self.logger.info(f"[DIAG] UDP diagnostics listening on 0.0.0.0:{self.port}")

    def _udp_loop(self) -> None:
        assert self._sock is not None
        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            text = data.decode("utf-8", errors="ignore").strip()
            parts = text.split()
            if len(parts) >= 3 and parts[0] == "HELLO":
                self._handle_hello(parts, addr)

    def _handle_hello(self, parts, addr) -> None:
        try:
            domain_id = int(parts[1])
        except ValueError:
            self.log("WARN", f"[DIAG] Ignoring HELLO with invalid domain from {addr}")
            return
        if domain_id != self.ros_domain_id or parts[2] != self.pairing_code:
            self.log("WARN", f"[DIAG] Ignoring HELLO domain/pairing mismatch from {addr}")
            return
        self.authorized_addr = addr
        self._send_text(f"ACK {self.ros_domain_id} {self.robot_name}")
        self.logger.info(f"[DIAG] Laptop diagnostics paired with {addr}")

    def log(self, level: str, message: str) -> None:
        level = level.upper()
        if level == "ERROR":
            self.logger.error(message)
        elif level in ("WARN", "WARNING"):
            self.logger.warn(message)
        else:
            self.logger.info(message)
        self._send_text(f"LOG {self.ros_domain_id} {self.robot_name} {level} {message}")

    def lidar(self, snapshot: DiagnosticSnapshot) -> None:
        sec = int(time.time())
        nsec = int((time.time() - sec) * 1_000_000_000)
        fields = [
            "LIDAR",
            str(self.ros_domain_id),
            self.robot_name,
            str(sec),
            str(nsec),
            snapshot.state,
            f"{snapshot.front:.3f}",
            f"{snapshot.left:.3f}",
            f"{snapshot.right:.3f}",
            f"{snapshot.nearest_dist:.3f}",
            f"{snapshot.nearest_angle:.1f}",
            f"{snapshot.gap_start:.1f}",
            f"{snapshot.gap_end:.1f}",
            f"{snapshot.turn_hint:.1f}",
            f"{snapshot.speed:.3f}",
            f"{snapshot.yaw:.3f}",
        ]
        self._send_text(" ".join(fields))

    def _send_text(self, text: str) -> None:
        if not self.enabled or self._sock is None or self.authorized_addr is None:
            return
        try:
            self._sock.sendto(text.encode("utf-8"), self.authorized_addr)
        except OSError as exc:
            self.logger.warn(f"[DIAG] UDP send failed: {exc}")

    def close(self) -> None:
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass


class PersistentJsonlLogger:
    """Append structured run records for post-run navigation debugging."""

    def __init__(self, path: str | Path, *, enabled: bool = True):
        self.path = Path(path)
        self.enabled = enabled
        self._warned = False
        if self.enabled:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
            except OSError:
                self.enabled = False

    def write(self, record: Dict[str, Any]) -> None:
        if not self.enabled:
            return
        payload = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            **record,
        }
        try:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, ensure_ascii=True, sort_keys=True) + "\n")
        except OSError:
            self._warned = True
