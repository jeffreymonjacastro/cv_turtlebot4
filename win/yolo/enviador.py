#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows)
#  - Envia WASD manual a home/ubuntu/original/recibidor.py por UDP 5007
#  - Lee output/signals/latest_signal.json
#  - Si hay senal izquierda/derecha estable, ejecuta giro automatico
# ============================================================
import json
import socket
import struct
import time
from pathlib import Path

import msvcrt

ROBOT_IP = "192.168.0.103"
ROBOT_PORT = 5007

SEND_HZ = 60

LIN = 2.00
MANUAL_ANG = 3.00
AUTO_TURN_ANG = 1.50
TURN_SECONDS = 1.05
SIGNAL_COOLDOWN_SECONDS = 3.0
SIGNAL_STALE_SECONDS = 2.0
CONF_THRESHOLD = 0.65

REPO_ROOT = Path(__file__).resolve().parents[2]
LATEST_SIGNAL_PATH = REPO_ROOT / "output" / "signals" / "latest_signal.json"


def pack_cmd(v, w):
    return struct.pack("ff", float(v), float(w))


def read_latest_signal():
    if not LATEST_SIGNAL_PATH.exists():
        return None

    try:
        payload = json.loads(LATEST_SIGNAL_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None

    direction = payload.get("direction")
    confidence = float(payload.get("confidence") or 0.0)
    timestamp = float(payload.get("timestamp") or 0.0)
    if direction not in ("left", "right", "none"):
        return None
    if direction == "none":
        return payload
    if confidence < CONF_THRESHOLD:
        return None
    if time.time() - timestamp > SIGNAL_STALE_SECONDS:
        return None
    return payload


def signal_event_id(signal):
    if not signal:
        return None
    direction = signal.get("direction")
    if direction not in ("left", "right"):
        return None
    return f"{direction}:{signal.get('timestamp')}"


def send_stop(sock):
    sock.sendto(pack_cmd(0.0, 0.0), (ROBOT_IP, ROBOT_PORT))


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    manual_v = 0.0
    manual_w = 0.0
    auto_direction = None
    auto_until = 0.0
    last_auto_start = 0.0
    consumed_signal_event = None

    period = 1.0 / SEND_HZ
    last_send = 0.0

    print("=== UDP Teleop + Signals Auto Turn (Windows) ===")
    print(f"Target: {ROBOT_IP}:{ROBOT_PORT} @ {SEND_HZ} Hz")
    print(f"Signal file: {LATEST_SIGNAL_PATH}")
    print("W: forward | S: back | A: left | D: right | X: stop/cancel | Q: quit")
    print("---------------------------------")

    try:
        while True:
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b"\x00", b"\xe0"):
                    _ = msvcrt.getch()
                    continue

                key = ch.decode(errors="ignore").lower()

                if key == "w":
                    manual_v, manual_w = +LIN, 0.0
                elif key == "s":
                    manual_v, manual_w = -LIN, 0.0
                elif key == "a":
                    manual_v, manual_w = 0.0, +MANUAL_ANG
                elif key == "d":
                    manual_v, manual_w = 0.0, -MANUAL_ANG
                elif key == "x":
                    manual_v, manual_w = 0.0, 0.0
                    auto_direction = None
                    auto_until = 0.0
                    consumed_signal_event = signal_event_id(read_latest_signal())
                    send_stop(sock)
                    print("\nSTOP enviado. Auto giro cancelado.")
                elif key == "q":
                    send_stop(sock)
                    print("\nSaliendo (STOP enviado).")
                    return

                if key in ("w", "a", "s", "d"):
                    print(
                        f"\r manual v={manual_v:+.2f} m/s | "
                        f"w={manual_w:+.2f} rad/s     ",
                        end="",
                    )

            now = time.time()

            if auto_direction and now >= auto_until:
                auto_direction = None
                manual_v, manual_w = 0.0, 0.0
                send_stop(sock)
                print("\n[AUTO] Giro terminado. STOP enviado.")

            if not auto_direction:
                signal = read_latest_signal()
                event_id = signal_event_id(signal)
                can_start = (
                    event_id is not None
                    and event_id != consumed_signal_event
                    and now - last_auto_start >= SIGNAL_COOLDOWN_SECONDS
                )
                if can_start:
                    auto_direction = signal["direction"]
                    auto_until = now + TURN_SECONDS
                    last_auto_start = now
                    consumed_signal_event = event_id
                    manual_v, manual_w = 0.0, 0.0
                    print(
                        f"\n[AUTO] Senal {auto_direction} "
                        f"conf={float(signal.get('confidence', 0.0)):.2f}. "
                        f"Girando {TURN_SECONDS:.2f}s."
                    )

            if auto_direction == "left":
                cmd_v, cmd_w = 0.0, +AUTO_TURN_ANG
            elif auto_direction == "right":
                cmd_v, cmd_w = 0.0, -AUTO_TURN_ANG
            else:
                cmd_v, cmd_w = manual_v, manual_w

            if now - last_send >= period:
                sock.sendto(pack_cmd(cmd_v, cmd_w), (ROBOT_IP, ROBOT_PORT))
                last_send = now

            time.sleep(0.001)

    finally:
        send_stop(sock)
        sock.close()


if __name__ == "__main__":
    main()
