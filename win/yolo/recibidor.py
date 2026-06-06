#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) - NO ejecutar en el TurtleBot
#  - Recibe SCAN / IMG desde home/ubuntu/signals/enviador.py
#  - Detecta senales de giro con un modelo YOLO custom
#  - Escribe output/signals/latest_signal.json para win/signals/enviador.py
# ============================================================
import base64
import json
import socket
import time
from pathlib import Path

import cv2
import numpy as np

# ========= Configuracion =========
ROBOT_IP = "192.168.0.103"
ROBOT_PORT = 6000

DESIRED_DOMAIN_ID = 2
PAIRING_CODE = "ROBOT_A_2"
EXPECTED_ROBOT_NAME = "turtlebot4_rensso_mora"

CONF_THRESHOLD = 0.65
IMG_SIZE = 640
SIGNAL_CHECK_EVERY_N_FRAMES = 3
STABLE_SIGNAL_FRAMES = 2
SCAN_PRINT_INTERVAL_SECONDS = 1.0

REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = REPO_ROOT / "models" / "signals" / "best.pt"
SIGNAL_OUTPUT_DIR = REPO_ROOT / "output" / "signals"
LATEST_SIGNAL_PATH = SIGNAL_OUTPUT_DIR / "latest_signal.json"

LEFT_ALIASES = ("left", "izquierda")
RIGHT_ALIASES = ("right", "derecha")

_last_scan_print = 0.0
_signal_frame_counter = 0
_candidate_direction = "none"
_candidate_count = 0
_stable_direction = "none"
_model = None


def should_print_scan():
    global _last_scan_print

    now = time.monotonic()
    if now - _last_scan_print < SCAN_PRINT_INTERVAL_SECONDS:
        return False
    _last_scan_print = now
    return True


def load_signal_model():
    if not MODEL_PATH.exists():
        print("[YOLO] Modelo no encontrado.")
        print(f"[YOLO] Coloca tu modelo entrenado en: {MODEL_PATH}")
        print("[YOLO] La ventana de camara funcionara, pero no habra deteccion.")
        return None

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[YOLO] Falta instalar ultralytics en la laptop.")
        print("       Ejemplo: python -m pip install ultralytics")
        print("[YOLO] La ventana de camara funcionara, pero no habra deteccion.")
        return None

    print(f"[YOLO] Cargando modelo: {MODEL_PATH}")
    return YOLO(str(MODEL_PATH))


def signal_direction_from_name(name):
    normalized = name.lower().replace("-", "_").replace(" ", "_")
    if any(alias in normalized for alias in LEFT_ALIASES):
        return "left"
    if any(alias in normalized for alias in RIGHT_ALIASES):
        return "right"
    return None


def write_latest_signal(direction, confidence, source_frame_time):
    SIGNAL_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "direction": direction,
        "confidence": round(float(confidence), 4),
        "timestamp": time.time(),
        "source_frame_time": source_frame_time,
    }
    tmp_path = LATEST_SIGNAL_PATH.with_suffix(".tmp")
    tmp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp_path.replace(LATEST_SIGNAL_PATH)


def update_stable_signal(direction, confidence, source_frame_time):
    global _candidate_direction, _candidate_count, _stable_direction

    if direction is None:
        _candidate_direction = "none"
        _candidate_count = 0
        if _stable_direction != "none":
            _stable_direction = "none"
            write_latest_signal("none", 0.0, source_frame_time)
            print("[SIGNAL] Senal perdida. Estado: none")
        return

    if direction == _candidate_direction:
        _candidate_count += 1
    else:
        _candidate_direction = direction
        _candidate_count = 1

    if _candidate_count >= STABLE_SIGNAL_FRAMES and direction != _stable_direction:
        _stable_direction = direction
        write_latest_signal(direction, confidence, source_frame_time)
        print(
            f"[SIGNAL] Detectado estable: {direction} "
            f"conf={confidence:.2f} frame_time={source_frame_time}"
        )


def draw_detection(img, detection):
    if detection is None:
        return

    x1, y1, x2, y2 = detection["box"]
    direction = detection["direction"]
    confidence = detection["confidence"]
    color = (0, 200, 255) if direction == "left" else (0, 255, 0)
    label = f"{direction} {confidence:.2f}"

    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        img,
        label,
        (x1, max(20, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


def detect_signal(img):
    if _model is None:
        return None

    try:
        results = _model.predict(
            source=img,
            imgsz=IMG_SIZE,
            conf=CONF_THRESHOLD,
            save=False,
            verbose=False,
        )
    except Exception as e:
        print(f"[YOLO] Error durante inferencia: {e}")
        return None

    if not results:
        return None

    result = results[0]
    if result.boxes is None or len(result.boxes) == 0:
        return None

    boxes = result.boxes
    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)
    names = result.names

    best = None
    for box, conf, cls_id in zip(xyxy, confs, classes):
        name = names.get(int(cls_id), f"class_{int(cls_id)}")
        direction = signal_direction_from_name(name)
        if direction is None:
            continue
        if best is None or float(conf) > best["confidence"]:
            x1, y1, x2, y2 = [int(v) for v in box]
            best = {
                "direction": direction,
                "confidence": float(conf),
                "name": name,
                "box": (x1, y1, x2, y2),
            }

    return best


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
    try:
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        n = int(parts[7])
        n_effective = min(n, len(parts[8:]))
        ranges = [float(r) for r in parts[8 : 8 + n_effective]]
        if should_print_scan():
            print(
                f"[SCAN] robot={robot_name} t={sec}.{nsec:09d} "
                f"n={n_effective} ejemplo={ranges[:5]}"
            )
    except ValueError as e:
        print(f"[SCAN] Error parseando: {e}")


def process_signal_from_image(img, sec, nsec):
    global _signal_frame_counter

    _signal_frame_counter += 1
    source_frame_time = f"{sec}.{nsec:09d}"
    if _signal_frame_counter % SIGNAL_CHECK_EVERY_N_FRAMES != 0:
        return

    detection = detect_signal(img)
    if detection is None:
        update_stable_signal(None, 0.0, source_frame_time)
        return

    draw_detection(img, detection)
    update_stable_signal(
        detection["direction"],
        detection["confidence"],
        source_frame_time,
    )


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

        process_signal_from_image(img, sec, nsec)

        cv2.imshow(f"Signals {robot_name} (domain {domain_id})", img)
        cv2.waitKey(1)
    except Exception as e:
        print(f"[IMG] Error: {e}")


def main():
    global _model

    _model = load_signal_model()
    write_latest_signal("none", 0.0, None)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (ROBOT_IP, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    print("[MAIN] Recibiendo telemetria de senales. Ctrl+C para salir.")
    print(f"[MAIN] Estado compartido: {LATEST_SIGNAL_PATH}")
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
        write_latest_signal("none", 0.0, None)
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
