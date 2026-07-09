#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) - NO ejecutar en el TurtleBot
#  - Recibe SCAN / IMG desde ubuntu/yolo/enviador.py
#  - Carga un YOLO detect custom y dibuja bounding boxes en vivo
#  - Escribe output/signals/latest_signal.json para win/yolo/enviador.py
# ============================================================
import base64
import json
import os
import socket
import time
from pathlib import Path

import cv2
import numpy as np

# ========= Telemetria =========
ROBOT_IP = "10.60.199.200"
ROBOT_PORT = 6000

DESIRED_DOMAIN_ID = 2
PAIRING_CODE = "ROBOT_A_2"
EXPECTED_ROBOT_NAME = "turtlebot4_rensso_mora"

# ========= YOLO =========
CONF_THRESHOLD = 0.30
ACTION_CONF_THRESHOLD = 0.90
ACTION_MIN_AREA_RATIO = 0.18
ACTION_CENTER_X_MIN = 0.25
ACTION_CENTER_X_MAX = 0.75
IMG_SIZE = 640
SIGNAL_CHECK_EVERY_N_FRAMES = 1
STABLE_SIGNAL_FRAMES = 2
MIN_SIGNAL_WRITE_INTERVAL_SECONDS = 0.10
SAVE_ACTIONABLE_COOLDOWN_SECONDS = 1.5
SCAN_PRINT_INTERVAL_SECONDS = 1.0

REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_CANDIDATES = [
    (
        Path(os.environ["YOLO_SIGNAL_MODEL"])
        if os.environ.get("YOLO_SIGNAL_MODEL")
        else None
    ),
    REPO_ROOT / "models" / "signals" / "best.pt",
    REPO_ROOT / "kaggle" / "v4" / "outputs" / "best.pt",
]
SIGNAL_OUTPUT_DIR = REPO_ROOT / "output" / "signals"
LATEST_SIGNAL_PATH = SIGNAL_OUTPUT_DIR / "latest_signal.json"
YOLO_OUTPUT_DIR = REPO_ROOT / "output" / "yolo"

LEFT_ALIASES = ("left", "izquierda")
RIGHT_ALIASES = ("right", "derecha")
STOP_ALIASES = ("stop", "alto", "detener")

_last_scan_print = 0.0
_signal_frame_counter = 0
_candidate_direction = "none"
_candidate_count = 0
_stable_direction = "none"
_model = None
_last_signal_write_time = 0.0
_last_signal_signature = None
_last_write_warning_time = 0.0
_last_saved_detection_time = 0.0
_last_saved_detection_direction = None


def should_print_scan():
    global _last_scan_print

    now = time.monotonic()
    if now - _last_scan_print < SCAN_PRINT_INTERVAL_SECONDS:
        return False
    _last_scan_print = now
    return True


def find_model_path():
    for path in MODEL_CANDIDATES:
        if path and path.exists():
            return path
    return None


def load_signal_model():
    model_path = find_model_path()
    if model_path is None:
        print("[YOLO] Modelo no encontrado.")
        print("[YOLO] Rutas probadas:")
        for candidate in MODEL_CANDIDATES:
            if candidate:
                print(f"       - {candidate}")
        print("[YOLO] Tambien puedes definir YOLO_SIGNAL_MODEL=C:\\ruta\\best.pt")
        print("[YOLO] La ventana de camara funcionara, pero no habra deteccion.")
        return None

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[YOLO] Falta instalar ultralytics en la laptop.")
        print("       Ejemplo: python -m pip install ultralytics")
        print("[YOLO] La ventana de camara funcionara, pero no habra deteccion.")
        return None

    print(f"[YOLO] Cargando modelo: {model_path}")
    model = YOLO(str(model_path))
    print(f"[YOLO] task={model.task} names={model.names}")
    return model


def signal_direction_from_name(name):
    normalized = name.lower().replace("-", "_").replace(" ", "_")
    if any(alias in normalized for alias in LEFT_ALIASES):
        return "left"
    if any(alias in normalized for alias in RIGHT_ALIASES):
        return "right"
    if any(alias in normalized for alias in STOP_ALIASES):
        return "stop"
    return None


def atomic_write_json(path, payload):
    global _last_write_warning_time

    text = json.dumps(payload, indent=2)
    tmp_path = path.with_name(f"{path.stem}.{os.getpid()}.{time.monotonic_ns()}.tmp")
    tmp_path.write_text(text, encoding="utf-8")

    for attempt in range(8):
        try:
            tmp_path.replace(path)
            return True
        except PermissionError:
            time.sleep(0.005 * (attempt + 1))

    # En Windows, el replace atomico falla si otro proceso esta leyendo el destino.
    # El lector ignora JSON parcial, asi que este fallback evita romper el loop de imagenes.
    for attempt in range(3):
        try:
            path.write_text(text, encoding="utf-8")
            try:
                tmp_path.unlink(missing_ok=True)
            except OSError:
                pass
            return True
        except PermissionError:
            time.sleep(0.01 * (attempt + 1))

    try:
        tmp_path.unlink(missing_ok=True)
    except OSError:
        pass

    now = time.monotonic()
    if now - _last_write_warning_time >= 1.0:
        print(f"[SIGNAL] Windows bloqueo {path}; se reintentara en el siguiente frame.")
        _last_write_warning_time = now
    return False


def write_latest_signal(direction, confidence, source_frame_time, detection=None):
    global _last_signal_signature, _last_signal_write_time

    SIGNAL_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    detection = detection or {}
    monotonic_now = time.monotonic()
    signature = (
        direction,
        round(float(confidence), 2),
        tuple(detection.get("box") or []),
        round(float(detection.get("area_ratio", 0.0)), 3),
        round(float(detection.get("center_x_ratio", 0.0)), 3),
        bool(detection.get("actionable", False)),
    )
    if (
        signature == _last_signal_signature
        and monotonic_now - _last_signal_write_time < MIN_SIGNAL_WRITE_INTERVAL_SECONDS
    ):
        return

    payload = {
        "direction": direction,
        "confidence": round(float(confidence), 4),
        "timestamp": time.time(),
        "source_frame_time": source_frame_time,
        "bbox_xyxy": detection.get("box"),
        "bbox_area_ratio": round(float(detection.get("area_ratio", 0.0)), 5),
        "bbox_center_x_ratio": round(float(detection.get("center_x_ratio", 0.0)), 5),
        "actionable": bool(detection.get("actionable", False)),
        "class_name": detection.get("name"),
        "thresholds": {
            "action_confidence": ACTION_CONF_THRESHOLD,
            "action_min_area_ratio": ACTION_MIN_AREA_RATIO,
            "action_center_x_min": ACTION_CENTER_X_MIN,
            "action_center_x_max": ACTION_CENTER_X_MAX,
            "stable_frames": STABLE_SIGNAL_FRAMES,
        },
    }
    if atomic_write_json(LATEST_SIGNAL_PATH, payload):
        _last_signal_signature = signature
        _last_signal_write_time = monotonic_now


def update_stable_signal(detection, source_frame_time):
    global _candidate_direction, _candidate_count, _stable_direction

    direction = detection["direction"] if detection else None
    confidence = detection["confidence"] if detection else 0.0

    if direction is None:
        _candidate_direction = "none"
        _candidate_count = 0
        if _stable_direction != "none":
            print("[SIGNAL] Senal perdida. Estado: none")
        _stable_direction = "none"
        write_latest_signal("none", 0.0, source_frame_time)
        return

    if direction == _candidate_direction:
        _candidate_count += 1
    else:
        _candidate_direction = direction
        _candidate_count = 1

    stable_direction = direction if _candidate_count >= STABLE_SIGNAL_FRAMES else "none"
    if stable_direction != _stable_direction:
        _stable_direction = stable_direction
        if stable_direction != "none":
            print(
                f"[SIGNAL] Estable: {stable_direction} "
                f"conf={confidence:.2f} area={detection['area_ratio']:.3f} "
                f"actionable={detection['actionable']} frame_time={source_frame_time}"
            )

    if stable_direction == "none":
        write_latest_signal("none", 0.0, source_frame_time, detection)
    else:
        write_latest_signal(stable_direction, confidence, source_frame_time, detection)


def detection_color(direction):
    if direction == "left":
        return (0, 200, 255)
    if direction == "right":
        return (0, 255, 0)
    if direction == "stop":
        return (0, 0, 255)
    return (255, 255, 255)


def draw_detection(img, detection):
    if detection is None:
        return

    x1, y1, x2, y2 = detection["box"]
    direction = detection["direction"]
    confidence = detection["confidence"]
    area_ratio = detection["area_ratio"]
    center_x_ratio = detection["center_x_ratio"]
    actionable = detection["actionable"]
    color = detection_color(direction)
    label = f"{direction} {confidence:.2f} {area_ratio:.0%}"
    if actionable:
        label += " ACTION"

    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    font_scale = 0.38
    thickness = 1
    (text_w, text_h), baseline = cv2.getTextSize(
        label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
    )
    label_x = max(0, min(x1, img.shape[1] - text_w - 4))
    label_y = max(text_h + 4, y1 - 5)
    cv2.rectangle(
        img,
        (label_x, label_y - text_h - baseline - 4),
        (label_x + text_w + 4, label_y + baseline),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        img,
        label,
        (label_x + 2, label_y - 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_status(img):
    if _stable_direction == "none":
        text = "Estado: buscando senal | comando esperado: adelante"
        color = (255, 255, 255)
    else:
        text = f"Estado estable: {_stable_direction}"
        color = detection_color(_stable_direction)
    cv2.putText(
        img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA
    )


def detect_signal(img):
    if _model is None:
        return None

    h, w = img.shape[:2]
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

        x1, y1, x2, y2 = [int(round(v)) for v in box]
        x1, x2 = max(0, min(w - 1, x1)), max(0, min(w - 1, x2))
        y1, y2 = max(0, min(h - 1, y1)), max(0, min(h - 1, y2))
        area_ratio = max(0, x2 - x1) * max(0, y2 - y1) / max(1, w * h)
        center_x_ratio = ((x1 + x2) / 2.0) / max(1, w)
        actionable = (
            float(conf) >= ACTION_CONF_THRESHOLD
            and area_ratio >= ACTION_MIN_AREA_RATIO
            and ACTION_CENTER_X_MIN <= center_x_ratio <= ACTION_CENTER_X_MAX
        )

        candidate = {
            "direction": direction,
            "confidence": float(conf),
            "name": name,
            "box": [x1, y1, x2, y2],
            "area_ratio": float(area_ratio),
            "center_x_ratio": float(center_x_ratio),
            "actionable": actionable,
        }
        if best is None or candidate["confidence"] > best["confidence"]:
            best = candidate

    return best


def save_actionable_detection(img, detection, source_frame_time):
    global _last_saved_detection_direction, _last_saved_detection_time

    if detection is None or not detection.get("actionable"):
        return

    now = time.monotonic()
    direction = detection["direction"]
    if (
        direction == _last_saved_detection_direction
        and now - _last_saved_detection_time < SAVE_ACTIONABLE_COOLDOWN_SECONDS
    ):
        return

    YOLO_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    safe_frame_time = str(source_frame_time).replace(".", "_")
    confidence = int(round(float(detection["confidence"]) * 100))
    area = int(round(float(detection["area_ratio"]) * 100))
    filename = (
        f"{time.strftime('%Y%m%d_%H%M%S')}_{safe_frame_time}_"
        f"{direction}_conf{confidence}_area{area}.jpg"
    )
    output_path = YOLO_OUTPUT_DIR / filename

    if cv2.imwrite(str(output_path), img):
        _last_saved_detection_direction = direction
        _last_saved_detection_time = now
        print(f"[YOLO] Captura guardada: {output_path}")
    else:
        print(f"[YOLO] No se pudo guardar captura: {output_path}")


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
    return
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
        draw_status(img)
        return

    detection = detect_signal(img)
    draw_detection(img, detection)
    update_stable_signal(detection, source_frame_time)
    draw_status(img)
    save_actionable_detection(img, detection, source_frame_time)


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

        cv2.imshow(f"YOLO signals {robot_name} (domain {domain_id})", img)
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

    print("[MAIN] Recibiendo frames y detectando senales. Ctrl+C para salir.")
    print(f"[MAIN] Estado compartido: {LATEST_SIGNAL_PATH}")
    print(
        "[MAIN] Para accionar: "
        f"conf>={ACTION_CONF_THRESHOLD:.2f}, area>={ACTION_MIN_AREA_RATIO:.1%}, "
        f"centro_x={ACTION_CENTER_X_MIN:.2f}-{ACTION_CENTER_X_MAX:.2f}, "
        f"estable {STABLE_SIGNAL_FRAMES} frames."
    )
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
