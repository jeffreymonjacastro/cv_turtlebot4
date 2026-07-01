#!/usr/bin/env python3
import os
import socket
import sys

ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.0.103")
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "6000"))
DESIRED_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "2"))
PAIRING_CODE = os.environ.get("PAIRING_CODE", "ROBOT_A_2")
EXPECTED_ROBOT_NAME = os.environ.get("ROBOT_NAME", "turtlebot4_rensso_mora")
SHOW_FULL_SCAN = os.environ.get("SHOW_FULL_SCAN", "0") == "1"


def robot_addr():
    ip = sys.argv[1] if len(sys.argv) > 1 else ROBOT_IP
    return ip, ROBOT_PORT


def do_handshake(sock, addr):
    sock.settimeout(1.0)
    hello = f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode("utf-8")
    print(f"[HANDSHAKE] Enviando HELLO a {addr}...")
    while True:
        sock.sendto(hello, addr)
        try:
            data, source = sock.recvfrom(4096)
        except socket.timeout:
            print("[HANDSHAKE] Sin ACK, reintentando...")
            continue
        parts = data.decode("utf-8", errors="ignore").strip().split()
        if len(parts) < 3 or parts[0] != "ACK":
            continue
        try:
            domain_id = int(parts[1])
        except ValueError:
            print(f"[HANDSHAKE] ACK invalido desde {source}: {' '.join(parts)}")
            continue
        robot_name = " ".join(parts[2:])
        if domain_id != DESIRED_DOMAIN_ID or robot_name != EXPECTED_ROBOT_NAME:
            print(f"[HANDSHAKE] ACK ignorado desde {source}: {' '.join(parts)}")
            continue
        print(f"[HANDSHAKE] Conectado a {robot_name} domain={domain_id}.")
        sock.settimeout(None)
        return


def handle_lidar(parts):
    if len(parts) < 16:
        print(f"[LIDAR] Mensaje corto: {' '.join(parts)}")
        return
    try:
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        state = parts[5]
        front, left, right = map(float, parts[6:9])
        nearest_dist = float(parts[9])
        nearest_angle = float(parts[10])
        gap_start = float(parts[11])
        gap_end = float(parts[12])
        turn_hint = float(parts[13])
        speed = float(parts[14])
        yaw = float(parts[15])
    except ValueError as e:
        print(f"[LIDAR] Error parseando: {e}")
        return
    print(
        f"[LIDAR] {robot_name} t={sec}.{nsec:09d} state={state:<11} "
        f"front={front:.2f}m left={left:.2f}m right={right:.2f}m "
        f"nearest={nearest_dist:.2f}m@{nearest_angle:+.0f}deg "
        f"gap=[{gap_start:+.0f},{gap_end:+.0f}] turn={turn_hint:+.0f}deg "
        f"cmd=({speed:.2f},{yaw:+.2f})"
    )


def handle_scan_array(parts):
    if len(parts) < 9:
        print(f"[SCAN_ARRAY] Mensaje corto: {' '.join(parts)}")
        return
    try:
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        angle_min = float(parts[5])
        angle_inc = float(parts[6])
        stride = int(parts[7])
        n = int(parts[8])
        values = parts[9 : 9 + n]
    except ValueError as e:
        print(f"[SCAN_ARRAY] Error parseando header: {e}")
        return

    if len(values) != n:
        print(f"[SCAN_ARRAY] n={n} pero llegaron {len(values)} valores.")

    shown = values if SHOW_FULL_SCAN else values[:16]
    suffix = "" if SHOW_FULL_SCAN or len(values) <= 16 else f" ... +{len(values) - 16}"
    print(
        f"[SCAN_ARRAY] {robot_name} t={sec}.{nsec:09d} "
        f"angle_min={angle_min:.3f} angle_inc={angle_inc:.4f} stride={stride} n={len(values)} "
        f"[{', '.join(shown)}{suffix}]"
    )


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = robot_addr()
    try:
        do_handshake(sock, addr)
        print("[MAIN] Recibiendo estados LIDAR. Ctrl+C para salir.")
        while True:
            data, _ = sock.recvfrom(65535)
            parts = data.decode("utf-8", errors="ignore").strip().split()
            if not parts:
                continue
            if parts[0] == "LIDAR":
                handle_lidar(parts)
            elif parts[0] == "SCAN_ARRAY":
                handle_scan_array(parts)
            elif parts[0] == "ACK":
                continue
            else:
                print(f"[MAIN] Mensaje desconocido: {parts[0]}")
    except KeyboardInterrupt:
        print("\n[MAIN] Cerrando...")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
