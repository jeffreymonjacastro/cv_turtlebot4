#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) - NO ejecutar en el TurtleBot
#  - Recibe SCAN / IMG desde enviador.py
#  - Procesa QR localmente desde las imagenes recibidas
#  - Para mover el robot usa controller_template.py por separado
# ============================================================
import socket
import base64
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import cv2

# ========= Configuracion =========
ROBOT_IP = os.environ.get("ROBOT_IP", "10.60.199.200")
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", os.environ.get("QR_TELEMETRY_PORT", "6611")))

DESIRED_DOMAIN_ID = 2
PAIRING_CODE = "ROBOT_A_2"
EXPECTED_ROBOT_NAME = "turtlebot4_rensso_mora"

SCAN_PRINT_INTERVAL_SECONDS = 1.0
QR_CHECK_EVERY_N_FRAMES = 3
REPO_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = REPO_ROOT / "output"

_last_scan_print = 0.0
_qr_frame_counter = 0
_last_qr_text = None
_qr_detector = cv2.QRCodeDetector()


def should_print_scan():
    global _last_scan_print

    now = time.monotonic()
    if now - _last_scan_print < SCAN_PRINT_INTERVAL_SECONDS:
        return False
    _last_scan_print = now
    return True


def save_qr_image(img, robot_name, sec, nsec, qr_text):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    safe_robot_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", robot_name).strip("_")
    if not safe_robot_name:
        safe_robot_name = "robot"

    safe_qr_text = re.sub(r"[^A-Za-z0-9_.-]+", "_", qr_text).strip("_")
    if not safe_qr_text:
        safe_qr_text = "qr"
    safe_qr_text = safe_qr_text[:40]

    filename = f"qr_{safe_robot_name}_{sec}_{nsec:09d}_{safe_qr_text}.jpg"
    output_path = OUTPUT_DIR / filename

    if not cv2.imwrite(str(output_path), img):
        raise RuntimeError(f"No se pudo guardar la imagen en {output_path}")

    return output_path


def do_handshake(sock: socket.socket, robot_addr):
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
            else:
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
    try:
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        n = int(parts[7])
        n_effective = min(n, len(parts[8:]))
        ranges = [float(r) for r in parts[8 : 8 + n_effective]]
        if should_print_scan():
            print(
                f"[SCAN] robot={robot_name} t={sec}.{nsec:09d} n={n_effective} ejemplo={ranges[:5]}"
            )
    except ValueError as e:
        print(f"[SCAN] Error parseando: {e}")


def process_qr_from_image(img, robot_name, sec, nsec):
    global _qr_frame_counter, _last_qr_text

    _qr_frame_counter += 1
    if _qr_frame_counter % QR_CHECK_EVERY_N_FRAMES != 0:
        return

    qr_text, points, _ = _qr_detector.detectAndDecode(img)
    full_img = img.copy()
    if points is not None:
        points = points.astype(int).reshape(-1, 2)
        for i in range(len(points)):
            cv2.line(
                img,
                tuple(points[i]),
                tuple(points[(i + 1) % len(points)]),
                (0, 255, 0),
                2,
            )

    if qr_text:
        if qr_text != _last_qr_text:
            _last_qr_text = qr_text
            output_path = save_qr_image(full_img, robot_name, sec, nsec, qr_text)
            print(
                f"\n[QR] *** robot={robot_name} t={sec}.{nsec:09d} contenido='{qr_text}' ***\n"
            )
            print(f"[QR] Imagen guardada en: {output_path}")
        cv2.putText(
            img,
            qr_text[:60],
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    else:
        _last_qr_text = None


def handle_img(parts):
    if len(parts) < 6:
        print("[IMG] Mensaje demasiado corto.")
        return
    try:
        robot_name = parts[2]
        domain_id = int(parts[1])
        sec = int(parts[3])
        nsec = int(parts[4])
        b64_str = " ".join(parts[5:])
        jpeg_bytes = base64.b64decode(b64_str)
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            print("[IMG] Error al decodificar imagen.")
            return

        process_qr_from_image(img, robot_name, sec, nsec)

        cv2.imshow(f"Camara {robot_name} (domain {domain_id})", img)
        cv2.waitKey(1)
    except Exception as e:
        print(f"[IMG] Error: {e}")


def handle_remote_qr(parts, raw_text):
    if len(parts) < 6:
        print("[QR] Mensaje demasiado corto.")
        return
    try:
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        qr_content = raw_text.split(" ", 5)[5]
        print(
            f"\n[QR REMOTO] robot={robot_name} t={sec}.{nsec:09d} contenido='{qr_content}'\n"
        )
    except (ValueError, IndexError) as e:
        print(f"[QR] Error parseando: {e}")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else ROBOT_IP
    robot_addr = (robot_ip, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    print("[MAIN] Recibiendo telemetria. Ctrl+C para salir.")
    print("[MAIN] QR se procesa localmente en esta laptop.")
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            text = data.decode("utf-8", errors="ignore")
            parts = text.split()

            if not parts:
                continue

            msg_type = parts[0]
            if msg_type == "SCAN":
                handle_scan(parts)
            elif msg_type == "IMG":
                handle_img(parts)
            elif msg_type == "QR":
                handle_remote_qr(parts, text)
            else:
                print(f"[MAIN] Mensaje desconocido desde {addr}: '{msg_type}'")

    except KeyboardInterrupt:
        print("\n[MAIN] Cerrando...")
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
