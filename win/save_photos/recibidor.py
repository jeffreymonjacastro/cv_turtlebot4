#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) - NO ejecutar en TurtleBot
#  - Recibe IMG desde ubuntu/save_photos/enviador.py o ubuntu/original/enviador.py
#  - Guarda 1 imagen por segundo en output/yolo-dataset
#  - Sirve para crear dataset de entrenamiento YOLO
# ============================================================
import base64
import socket
import time
from pathlib import Path

import cv2
import numpy as np

ROBOT_IP = "192.168.0.108"
ROBOT_PORT = 6000

DESIRED_DOMAIN_ID = 2
PAIRING_CODE = "ROBOT_A_2"
EXPECTED_ROBOT_NAME = "turtlebot4_rensso_mora"

SAVE_INTERVAL_SECONDS = 1.0
SCAN_PRINT_INTERVAL_SECONDS = 2.0

REPO_ROOT = Path(__file__).resolve().parents[2]
DATASET_DIR = REPO_ROOT / "output" / "yolo-dataset-v5"

_last_save_time = 0.0
_last_scan_print = 0.0


def next_saved_count():
    counts = []
    for path in DATASET_DIR.glob("frame_*.jpg"):
        try:
            counts.append(int(path.stem.split("_", 1)[1]))
        except ValueError:
            pass
    return max(counts, default=0)


_saved_count = next_saved_count()


def should_print_scan():
    global _last_scan_print

    now = time.monotonic()
    if now - _last_scan_print < SCAN_PRINT_INTERVAL_SECONDS:
        return False
    _last_scan_print = now
    return True


def save_frame_if_due(img, robot_name, sec, nsec):
    global _last_save_time, _saved_count

    now = time.monotonic()
    if now - _last_save_time < SAVE_INTERVAL_SECONDS:
        return

    DATASET_DIR.mkdir(parents=True, exist_ok=True)
    _saved_count += 1
    filename = f"frame_{_saved_count:06d}.jpg"
    output_path = DATASET_DIR / filename

    if not cv2.imwrite(str(output_path), img):
        print(f"[SAVE] Error guardando: {output_path}")
        _saved_count -= 1
        return

    _last_save_time = now
    print(f"[SAVE] {output_path}")


def do_handshake(sock, robot_addr):
    sock.settimeout(1.0)
    print(f"[HANDSHAKE] Iniciando con {robot_addr}...")
    while True:
        msg = f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode("utf-8")
        sock.sendto(msg, robot_addr)
        try:
            data, addr = sock.recvfrom(4096)
            text = data.decode("utf-8").strip()
            parts = text.split()

            if len(parts) >= 3 and parts[0] == "ACK":
                domain_str = parts[1]
                robot_name = " ".join(parts[2:])
                print(f"[HANDSHAKE] Recibido: '{text}' desde {addr}")

                try:
                    domain_id = int(domain_str)
                except ValueError:
                    print("[HANDSHAKE] domain_id invalido, reintentando...")
                    continue

                if domain_id != DESIRED_DOMAIN_ID:
                    print("[HANDSHAKE] ROS_DOMAIN_ID no coincide. Reintentando...")
                    continue

                if robot_name != EXPECTED_ROBOT_NAME:
                    print("[HANDSHAKE] robot_name no coincide. Reintentando...")
                    continue

                print(
                    f"[HANDSHAKE] Emparejado con '{robot_name}' (domain {domain_id})."
                )
                sock.settimeout(None)
                return

            print(f"[HANDSHAKE] Mensaje inesperado: '{text}', reintentando...")

        except socket.timeout:
            print("[HANDSHAKE] Timeout esperando ACK, reintentando...")
        except KeyboardInterrupt:
            print("[HANDSHAKE] Cancelado.")
            raise


def handle_scan(parts):
    if len(parts) < 8:
        print("[SCAN] Mensaje demasiado corto.")
        return

    if not should_print_scan():
        return

    try:
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        n = int(parts[7])
        n_effective = min(n, len(parts[8:]))
        print(f"[SCAN] robot={robot_name} t={sec}.{nsec:09d} n={n_effective}")
    except ValueError as e:
        print(f"[SCAN] Error parseando: {e}")


def handle_img(parts):
    if len(parts) < 6:
        print("[IMG] Mensaje demasiado corto.")
        return

    try:
        domain_id = int(parts[1])
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        b64_str = " ".join(parts[5:])

        jpeg_bytes = base64.b64decode(b64_str)
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            print("[IMG] Error al decodificar imagen.")
            return

        save_frame_if_due(img, robot_name, sec, nsec)

        cv2.imshow(f"Save photos {robot_name} (domain {domain_id})", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt

    except KeyboardInterrupt:
        raise
    except Exception as e:
        print(f"[IMG] Error: {e}")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (ROBOT_IP, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    print("[MAIN] Guardando 1 imagen/segundo. Ctrl+C o Q para salir.")
    print(f"[MAIN] Dataset: {DATASET_DIR}")
    try:
        while True:
            data, _addr = sock.recvfrom(65535)
            text = data.decode("utf-8", errors="ignore")
            parts = text.split()

            if not parts:
                continue

            msg_type = parts[0]
            if msg_type == "SCAN":
                handle_scan(parts)
            elif msg_type == "IMG":
                handle_img(parts)
            else:
                print(f"[MAIN] Mensaje desconocido: '{msg_type}'")

    except KeyboardInterrupt:
        print("\n[MAIN] Cerrando...")
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
