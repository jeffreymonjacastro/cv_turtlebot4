#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) - NO ejecutar en el TurtleBot
#  - Recibe SCAN / IMG desde ubuntu/yolo/enviador.py
#  - Carga un YOLO detect custom y dibuja bounding boxes en vivo
#  - Escribe output/signals/latest_signal.json para win/yolo/enviador.py
# ============================================================
import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from win.yolo.frame_stream import decode_img_parts, do_handshake as frame_stream_handshake
from win.yolo.perception_event_state import PerceptionEventState
from win.yolo.qr_pipeline import LatestFrameQRWorker, QRFrame
from win.yolo.qr_validator import QRValidator
from win.yolo.qr_zxing import DEFAULT_VARIANTS, ZXingQRDecoder

# ========= Telemetria =========
ROBOT_IP = os.environ.get("ROBOT_IP", "10.60.199.200")
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", os.environ.get("YOLO_TELEMETRY_PORT", "6610")))

DESIRED_DOMAIN_ID = 2
PAIRING_CODE = "ROBOT_A_2"
EXPECTED_ROBOT_NAME = "turtlebot4_rensso_mora"

# ========= YOLO =========
CONF_THRESHOLD = 0.30
ACTION_CONF_THRESHOLD = float(os.environ.get("YOLO_ACTION_CONF_THRESHOLD", "0.65"))
ACTION_MIN_AREA_RATIO = float(os.environ.get("YOLO_ACTION_MIN_AREA_RATIO", "0.02"))
ACTION_CENTER_X_MIN = float(os.environ.get("YOLO_ACTION_CENTER_X_MIN", "0.10"))
ACTION_CENTER_X_MAX = float(os.environ.get("YOLO_ACTION_CENTER_X_MAX", "0.90"))
IMG_SIZE = 640
SIGNAL_CHECK_EVERY_N_FRAMES = 1
STABLE_SIGNAL_FRAMES = 2
MIN_SIGNAL_WRITE_INTERVAL_SECONDS = 0.10
SCAN_PRINT_INTERVAL_SECONDS = 1.0

MODEL_CANDIDATES = [
    (
        Path(os.environ["YOLO_SIGNAL_MODEL"])
        if os.environ.get("YOLO_SIGNAL_MODEL")
        else None
    ),
    REPO_ROOT / "models" / "signals" / "best.pt",
    REPO_ROOT / "kaggle" / "v3" / "outputs" / "best.pt",
]
SIGNAL_OUTPUT_DIR = REPO_ROOT / "output" / "signals"
LATEST_SIGNAL_PATH = SIGNAL_OUTPUT_DIR / "latest_signal.json"
LATEST_QR_EVENT_PATH = SIGNAL_OUTPUT_DIR / "latest_qr_event.json"

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
_view_only = False
_event_state = None
_qr_worker = None
_qr_latest = {"status": "disabled"}
_perception_log_path = None


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
    try:
        if _event_state is not None:
            _event_state.write_yolo(payload)
            wrote = True
        else:
            wrote = atomic_write_json(LATEST_SIGNAL_PATH, payload)
    except OSError as exc:
        print(f"[SIGNAL] Error escribiendo {LATEST_SIGNAL_PATH}: {exc}")
        wrote = False
    if wrote:
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
    label = (
        f"{direction} {confidence:.2f} area={area_ratio:.1%} cx={center_x_ratio:.2f}"
    )
    if actionable:
        label += " ACTION"

    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        img,
        label,
        (x1, max(20, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def draw_status(img):
    if _view_only:
        text = "Vista en vivo oficial | YOLO desactivado (--view-only)"
        color = (255, 255, 255)
    elif _stable_direction == "none":
        text = "Estado: buscando senal | comando esperado: adelante"
        color = (255, 255, 255)
    else:
        text = f"Estado estable: {_stable_direction}"
        color = detection_color(_stable_direction)
    cv2.putText(
        img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA
    )


def _update_qr_latest(record):
    global _qr_latest
    _qr_latest = dict(record)


def draw_qr_status(img):
    status = str(_qr_latest.get("qr_event_status") or _qr_latest.get("status") or "idle")
    payload = str(_qr_latest.get("qr_event") or "NONE")
    variant = str(_qr_latest.get("qr_decode_variant") or "none")
    latency = float(_qr_latest.get("qr_decode_latency_ms") or 0.0)
    progress = str(_qr_latest.get("qr_confirmation_progress") or "0/0")
    reason = str(_qr_latest.get("qr_rejection_reason") or "none")
    metrics = _qr_latest.get("queue_metrics") or {}
    color = (0, 255, 0) if status == "validated" else (0, 200, 255)
    lines = [
        f"QR {status} payload={payload[:36]} progress={progress}",
        f"variant={variant} latency={latency:.1f}ms reason={reason[:32]}",
        f"queue replaced={metrics.get('replaced', 0)} stale={metrics.get('stale', 0)} errors={metrics.get('errors', 0)}",
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            img,
            text,
            (10, 50 + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )


def _append_perception_record(record):
    if _perception_log_path is None:
        return
    _perception_log_path.parent.mkdir(parents=True, exist_ok=True)
    with _perception_log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=True) + "\n")


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


def do_handshake(sock, robot_addr):
    print(f"[HANDSHAKE] Iniciando con {robot_addr}...")
    try:
        frame_stream_handshake(
            sock,
            robot_addr,
            domain_id=DESIRED_DOMAIN_ID,
            pairing_code=PAIRING_CODE,
            expected_robot_name=EXPECTED_ROBOT_NAME,
        )
    except KeyboardInterrupt:
        print("[HANDSHAKE] Cancelado.")
        raise
    print(f"[HANDSHAKE] Emparejado con '{EXPECTED_ROBOT_NAME}' (domain {DESIRED_DOMAIN_ID}).")


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

    if _view_only:
        draw_status(img)
        return

    _signal_frame_counter += 1
    source_frame_time = f"{sec}.{nsec:09d}"
    if _signal_frame_counter % SIGNAL_CHECK_EVERY_N_FRAMES != 0:
        draw_status(img)
        return

    detection = detect_signal(img)
    draw_detection(img, detection)
    update_stable_signal(detection, source_frame_time)
    draw_status(img)


def handle_img(parts):
    if len(parts) < 6:
        print("[IMG] Mensaje demasiado corto.")
        return
    try:
        frame = decode_img_parts(parts)
        img = frame.image
        if _qr_worker is not None:
            _qr_worker.submit(
                QRFrame(
                    image=img.copy(),
                    frame_id=frame.frame_id,
                    source_frame_time=frame.source_frame_time,
                    received_at=frame.received_at,
                    received_monotonic=frame.received_monotonic,
                )
            )

        yolo_started = time.perf_counter()
        process_signal_from_image(img, frame.sec, frame.nanosec)
        yolo_loop_ms = (time.perf_counter() - yolo_started) * 1000.0
        _append_perception_record(
            {
                "record_type": "yolo_frame",
                "timestamp": time.time(),
                "source_frame_time": frame.source_frame_time,
                "yolo_loop_latency_ms": round(yolo_loop_ms, 3),
                "stable_direction": _stable_direction,
                "qr_enabled": _qr_worker is not None,
            }
        )
        if _qr_worker is not None:
            draw_qr_status(img)

        cv2.imshow(f"YOLO signals {frame.robot_name} (domain {frame.domain_id})", img)
        cv2.waitKey(1)
    except Exception as e:
        print(f"[IMG] Error: {e}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ip", nargs="?", default=ROBOT_IP, help="IP del TurtleBot")
    parser.add_argument(
        "--view-only",
        action="store_true",
        help="Solo muestra la camara en vivo. No ejecuta YOLO ni escribe latest_signal.json.",
    )
    parser.add_argument("--enable-qr", action="store_true", help="Enable isolated laptop ZXing QR processing.")
    parser.add_argument("--qr-event-path", default=str(LATEST_QR_EVENT_PATH), help="Validated QR semantic state path.")
    parser.add_argument("--perception-log", default=None, help="Optional combined laptop perception JSONL log.")
    parser.add_argument("--qr-max-hz", type=float, default=2.0)
    parser.add_argument("--qr-max-frame-age-s", type=float, default=0.5)
    parser.add_argument("--qr-confirm-count", type=int, default=2)
    parser.add_argument("--qr-confirm-window-s", type=float, default=1.2)
    parser.add_argument("--qr-duplicate-cooldown-s", type=float, default=30.0)
    parser.add_argument(
        "--qr-variants",
        default=",".join(DEFAULT_VARIANTS),
        help="Comma-separated bounded ZXing preprocessing stages.",
    )
    return parser.parse_args()


def print_banner(robot_ip):
    mode = "VIEW ONLY" if _view_only else "LIVE VIEW + YOLO"
    print("=" * 72)
    print("[CAMERA] Ventana oficial de vista en vivo del TurtleBot para el operador")
    print(f"[CAMERA] Modo: {mode}")
    print(f"[CAMERA] Robot: {robot_ip}:{ROBOT_PORT}")
    print(f"[CAMERA] Estado compartido: {LATEST_SIGNAL_PATH}")
    print("=" * 72)


def main():
    global _event_state, _model, _perception_log_path, _qr_latest, _qr_worker, _view_only

    args = parse_args()
    _view_only = args.view_only
    robot_ip = args.robot_ip
    _perception_log_path = Path(args.perception_log).expanduser().resolve() if args.perception_log else None
    qr_event_path = Path(args.qr_event_path).expanduser().resolve()
    _event_state = PerceptionEventState(yolo_path=LATEST_SIGNAL_PATH, qr_path=qr_event_path)

    if args.enable_qr:
        try:
            qr_event_path.unlink(missing_ok=True)
        except OSError:
            pass
        variants = tuple(item.strip() for item in args.qr_variants.split(",") if item.strip())
        decoder = ZXingQRDecoder(variants)
        validator = QRValidator(
            confirm_count=args.qr_confirm_count,
            confirm_window_s=args.qr_confirm_window_s,
            duplicate_cooldown_s=args.qr_duplicate_cooldown_s,
            max_frame_age_s=args.qr_max_frame_age_s,
        )
        _qr_worker = LatestFrameQRWorker(
            event_state=_event_state,
            decoder=decoder,
            validator=validator,
            max_hz=args.qr_max_hz,
            log_path=_perception_log_path,
            result_callback=_update_qr_latest,
        )
        _qr_worker.start()
        _qr_latest = {"status": "starting" if decoder.available else "decoder_unavailable"}
        print(f"[QR] ZXing enabled={decoder.available} event={qr_event_path} variants={variants}")
        if not decoder.available:
            print(f"[QR] ZXing unavailable; YOLO will continue: {decoder.import_error}")

    if _view_only:
        _model = None
    else:
        _model = load_signal_model()
        write_latest_signal("none", 0.0, None)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (robot_ip, ROBOT_PORT)

    print_banner(robot_ip)
    do_handshake(sock, robot_addr)

    if _view_only:
        print("[MAIN] Recibiendo frames. YOLO desactivado por --view-only. Ctrl+C para salir.")
    else:
        print("[MAIN] Recibiendo frames y detectando senales. Ctrl+C para salir.")
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
        if _qr_worker is not None:
            _qr_worker.stop()
        if not _view_only:
            write_latest_signal("none", 0.0, None)
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
