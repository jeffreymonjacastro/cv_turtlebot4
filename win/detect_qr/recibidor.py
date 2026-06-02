#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) — NO ejecutar en el TurtleBot
#  - Recibe telemetría (SCAN / IMG / QR) desde enviador.py
#  - Para mover el robot usa controller_template.py por separado
# ============================================================
import socket
import base64

import numpy as np
import cv2

# ========= Configuración =========
ROBOT_IP   = "192.168.0.103"
ROBOT_PORT = 6000

DESIRED_DOMAIN_ID   = 2
PAIRING_CODE        = "ROBOT_A_2"
EXPECTED_ROBOT_NAME = "turtlebot4_rensso_mora"


def do_handshake(sock: socket.socket, robot_addr):
    sock.settimeout(1.0)
    print(f"[HANDSHAKE] Iniciando con {robot_addr}...")
    while True:
        msg = f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode("utf-8")
        sock.sendto(msg, robot_addr)
        try:
            data, addr = sock.recvfrom(4096)
            text  = data.decode("utf-8").strip()
            parts = text.split()

            if len(parts) >= 3 and parts[0] == "ACK":
                domain_str = parts[1]
                robot_name = " ".join(parts[2:])
                print(f"[HANDSHAKE] Recibido: '{text}' desde {addr}")
                try:
                    domain_id = int(domain_str)
                except ValueError:
                    print("[HANDSHAKE] domain_id inválido, reintentando...")
                    continue
                if domain_id != DESIRED_DOMAIN_ID:
                    print(f"[HANDSHAKE] ROS_DOMAIN_ID no coincide. Reintentando...")
                    continue
                if robot_name != EXPECTED_ROBOT_NAME:
                    print(f"[HANDSHAKE] robot_name no coincide. Reintentando...")
                    continue
                print(f"[HANDSHAKE] Emparejado con '{robot_name}' (domain {domain_id}).")
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
        robot_name  = parts[2]
        sec         = int(parts[3])
        nsec        = int(parts[4])
        n           = int(parts[7])
        n_effective = min(n, len(parts[8:]))
        ranges      = [float(r) for r in parts[8:8 + n_effective]]
        print(f"[SCAN] robot={robot_name} t={sec}.{nsec:09d} n={n_effective} ejemplo={ranges[:5]}")
    except ValueError as e:
        print(f"[SCAN] Error parseando: {e}")


def handle_img(parts):
    if len(parts) < 6:
        print("[IMG] Mensaje demasiado corto.")
        return
    try:
        robot_name = parts[2]
        domain_id  = int(parts[1])
        b64_str    = " ".join(parts[5:])
        jpeg_bytes = base64.b64decode(b64_str)
        arr        = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img        = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            print("[IMG] Error al decodificar imagen.")
            return
        cv2.imshow(f"Camara {robot_name} (domain {domain_id})", img)
        cv2.waitKey(1)
    except Exception as e:
        print(f"[IMG] Error: {e}")


def handle_qr(parts, raw_text):
    if len(parts) < 6:
        print("[QR] Mensaje demasiado corto.")
        return
    try:
        robot_name = parts[2]
        sec        = int(parts[3])
        nsec       = int(parts[4])
        qr_content = raw_text.split(" ", 5)[5]
        print(f"\n[QR] *** robot={robot_name} t={sec}.{nsec:09d} contenido='{qr_content}' ***\n")
    except (ValueError, IndexError) as e:
        print(f"[QR] Error parseando: {e}")


def main():
    sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (ROBOT_IP, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    print("[MAIN] Recibiendo telemetría. Ctrl+C para salir.")
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            text       = data.decode("utf-8", errors="ignore")
            parts      = text.split()

            if not parts:
                continue

            msg_type = parts[0]
            if msg_type == "SCAN":
                handle_scan(parts)
            elif msg_type == "IMG":
                handle_img(parts)
            elif msg_type == "QR":
                handle_qr(parts, text)
            else:
                print(f"[MAIN] Mensaje desconocido desde {addr}: '{msg_type}'")

    except KeyboardInterrupt:
        print("\n[MAIN] Cerrando...")
    finally:
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
