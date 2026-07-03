#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows)
#  - Envia comandos UDP a ubuntu/original/recibidor.py por puerto 5007
#  - Lee output/signals/latest_signal.json escrito por win/yolo/recibidor.py
#  - Por defecto avanza lento; si la bbox es grande, gira 90 grados o se detiene
# ============================================================
import json
import math
import socket
import struct
import time
from pathlib import Path

import msvcrt

ROBOT_IP = "172.31.245.200"
ROBOT_PORT = 5007

SEND_HZ = 30
SIGNAL_READ_HZ = 10
FORWARD_SPEED = 0.08
TURN_ANGULAR_SPEED = 0.45
TURN_ANGLE_DEGREES = 90
TURN_SECONDS = math.radians(TURN_ANGLE_DEGREES) / TURN_ANGULAR_SPEED
STOP_SECONDS = 2.0
ACTION_COOLDOWN_SECONDS = 1.0
SIGNAL_STALE_SECONDS = 0.8
ACTION_CONF_THRESHOLD = 0.90
ACTION_MIN_AREA_RATIO = 0.18
ACTION_CENTER_X_MIN = 0.25
ACTION_CENTER_X_MAX = 0.75

REPO_ROOT = Path(__file__).resolve().parents[2]
LATEST_SIGNAL_PATH = REPO_ROOT / "output" / "signals" / "latest_signal.json"


def pack_cmd(v, w):
    return struct.pack("ff", float(v), float(w))


def send_cmd(sock, v, w):
    sock.sendto(pack_cmd(v, w), (ROBOT_IP, ROBOT_PORT))


def send_stop(sock):
    send_cmd(sock, 0.0, 0.0)


def read_latest_signal():
    if not LATEST_SIGNAL_PATH.exists():
        return None

    try:
        payload = json.loads(LATEST_SIGNAL_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, ValueError):
        return None

    direction = payload.get("direction")
    if direction not in ("left", "right", "stop", "none"):
        return None
    if direction == "none":
        return None

    confidence = float(payload.get("confidence") or 0.0)
    area_ratio = float(payload.get("bbox_area_ratio") or 0.0)
    center_x_ratio = float(payload.get("bbox_center_x_ratio") or 0.0)
    timestamp = float(payload.get("timestamp") or 0.0)
    actionable = bool(payload.get("actionable", False))

    if time.time() - timestamp > SIGNAL_STALE_SECONDS:
        return None
    if confidence < ACTION_CONF_THRESHOLD:
        return None
    if area_ratio < ACTION_MIN_AREA_RATIO:
        return None
    if not (ACTION_CENTER_X_MIN <= center_x_ratio <= ACTION_CENTER_X_MAX):
        return None
    if not actionable:
        return None

    return payload


def signal_event_id(signal):
    if not signal:
        return None
    return f"{signal.get('direction')}:{signal.get('source_frame_time')}:{signal.get('timestamp')}"


def print_status(mode, paused, last_signal):
    if paused:
        state = "PAUSA"
    elif mode:
        state = mode
    else:
        state = f"adelante v={FORWARD_SPEED:.2f}"

    if last_signal:
        direction = last_signal.get("direction")
        conf = float(last_signal.get("confidence") or 0.0)
        area = float(last_signal.get("bbox_area_ratio") or 0.0)
        center_x = float(last_signal.get("bbox_center_x_ratio") or 0.0)
        print(
            f"\r[AUTO] {state} | senal={direction} conf={conf:.2f} area={area:.1%} cx={center_x:.2f}       ",
            end="",
        )
    else:
        print(f"\r[AUTO] {state} | sin senal accionable       ", end="")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / SEND_HZ
    last_send = 0.0
    last_print = 0.0
    last_signal_read = 0.0
    last_signal = None

    mode = None
    mode_until = 0.0
    cooldown_until = 0.0
    consumed_signal_event = None
    paused = False

    print("=== YOLO Signals Auto Driver (Windows) ===")
    print(f"Target cmd_vel UDP: {ROBOT_IP}:{ROBOT_PORT} @ {SEND_HZ} Hz")
    print(f"Signal file: {LATEST_SIGNAL_PATH}")
    print(f"Forward: {FORWARD_SPEED:.2f} m/s")
    print(
        f"Turns: {TURN_ANGLE_DEGREES} deg, w={TURN_ANGULAR_SPEED:.2f} rad/s, "
        f"duration={TURN_SECONDS:.2f}s"
    )
    print(f"Stop signal: stop for {STOP_SECONDS:.1f}s")
    print("X: pausa/reanudar | Q: salir con STOP")
    print("---------------------------------")

    try:
        while True:
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b"\x00", b"\xe0"):
                    _ = msvcrt.getch()
                    continue

                key = ch.decode(errors="ignore").lower()
                if key == "x":
                    paused = not paused
                    mode = None
                    mode_until = 0.0
                    send_stop(sock)
                    print(
                        "\n[MANUAL] Pausa activada."
                        if paused
                        else "\n[MANUAL] Reanudando avance automatico."
                    )
                elif key == "q":
                    send_stop(sock)
                    print("\n[MAIN] Saliendo (STOP enviado).")
                    return

            now = time.time()
            if now - last_signal_read >= 1.0 / SIGNAL_READ_HZ:
                last_signal = read_latest_signal()
                last_signal_read = now

            if mode and now >= mode_until:
                print(f"\n[AUTO] Accion terminada: {mode}.")
                mode = None
                mode_until = 0.0
                cooldown_until = now + ACTION_COOLDOWN_SECONDS
                send_stop(sock)

            if not paused and mode is None and now >= cooldown_until:
                event_id = signal_event_id(last_signal)
                if event_id is not None and event_id != consumed_signal_event:
                    consumed_signal_event = event_id
                    direction = last_signal["direction"]
                    conf = float(last_signal.get("confidence") or 0.0)
                    area = float(last_signal.get("bbox_area_ratio") or 0.0)
                    center_x = float(last_signal.get("bbox_center_x_ratio") or 0.0)
                    if direction == "left":
                        mode = "turn_left"
                        mode_until = now + TURN_SECONDS
                    elif direction == "right":
                        mode = "turn_right"
                        mode_until = now + TURN_SECONDS
                    elif direction == "stop":
                        mode = "stop"
                        mode_until = now + STOP_SECONDS

                    if mode:
                        print(
                            f"\n[AUTO] Accion por senal {direction}: {mode} "
                            f"conf={conf:.2f} area={area:.1%} cx={center_x:.2f}"
                        )

            if paused:
                cmd_v, cmd_w = 0.0, 0.0
            elif mode == "turn_left":
                cmd_v, cmd_w = 0.0, +TURN_ANGULAR_SPEED
            elif mode == "turn_right":
                cmd_v, cmd_w = 0.0, -TURN_ANGULAR_SPEED
            elif mode == "stop":
                cmd_v, cmd_w = 0.0, 0.0
            else:
                cmd_v, cmd_w = FORWARD_SPEED, 0.0

            if now - last_send >= period:
                send_cmd(sock, cmd_v, cmd_w)
                last_send = now

            if now - last_print >= 0.5:
                print_status(mode, paused, last_signal)
                last_print = now

            time.sleep(0.002)

    finally:
        send_stop(sock)
        sock.close()


if __name__ == "__main__":
    main()
