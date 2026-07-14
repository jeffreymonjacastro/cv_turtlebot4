#!/usr/bin/env python3
# ============================================================
#  SCRIPT DE LAPTOP (Windows) - NO ejecutar en el TurtleBot
#  - Recibe SCAN / IMG desde home/ubuntu/handcraft/enviador.py
#  - Detecta QR checkpoints con OpenCV QRCodeDetector
#  - Detecta flechas izquierda/derecha con vision handcrafted
#  - Escribe output/handcraft/latest_signal.json para win/handcraft/enviador.py
# ============================================================
import base64
import json
import os
import socket
import time
from pathlib import Path

import cv2
import numpy as np

# ========= Configuracion =========
ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")
ROBOT_PORT = 6000

DESIRED_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "2"))
PAIRING_CODE = os.environ.get("PAIRING_CODE", "ROBOT_PAIRING_CODE")
EXPECTED_ROBOT_NAME = os.environ.get("ROBOT_NAME", "turtlebot4")

ARROW_SCORE_THRESHOLD = 0.65
ARROW_STABLE_FRAMES = 2
VISION_CHECK_EVERY_N_FRAMES = 2
SCAN_PRINT_INTERVAL_SECONDS = 1.0

MIN_COMPONENT_AREA = 500
MIN_COMPONENT_EXTENT = 0.18
MIN_BOX_FRACTION = 0.04
EDGE_BAND_FRACTION = 0.12
MIN_SIGN_RADIUS_FRACTION = 0.10
MAX_SIGN_RADIUS_FRACTION = 0.48
MIN_SIGN_BRIGHT_RATIO = 0.35
MIN_CIRCLE_CIRCULARITY = 0.42
CIRCLE_INNER_SCALE = 0.72
MAX_SIGN_CANDIDATES = 5
MIN_HEAD_SCORE_RATIO = 1.08

REPO_ROOT = Path(__file__).resolve().parents[2]
HANDCRAFT_OUTPUT_DIR = REPO_ROOT / "output" / "handcraft"
LATEST_SIGNAL_PATH = HANDCRAFT_OUTPUT_DIR / "latest_signal.json"

_last_scan_print = 0.0
_vision_frame_counter = 0
_candidate_direction = "none"
_candidate_count = 0
_stable_direction = "none"
_last_qr_text = None
_qr_detector = cv2.QRCodeDetector()


def should_print_scan():
    global _last_scan_print

    now = time.monotonic()
    if now - _last_scan_print < SCAN_PRINT_INTERVAL_SECONDS:
        return False
    _last_scan_print = now
    return True


def write_latest_signal(signal_type, direction, qr_text, confidence, source_frame_time):
    HANDCRAFT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "type": signal_type,
        "direction": direction,
        "qr_text": qr_text,
        "confidence": round(float(confidence), 4),
        "timestamp": time.time(),
        "source_frame_time": source_frame_time,
    }
    tmp_path = LATEST_SIGNAL_PATH.with_suffix(".tmp")
    tmp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp_path.replace(LATEST_SIGNAL_PATH)


def preprocess_dark_mask(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(
        blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def bright_ratio(gray, box):
    x1, y1, x2, y2 = box
    roi = gray[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0
    return float(np.count_nonzero(roi > 150)) / float(roi.size)


def add_sign_candidate(candidates, gray, cx, cy, radius, source):
    height, width = gray.shape[:2]
    min_radius = min(height, width) * MIN_SIGN_RADIUS_FRACTION
    max_radius = min(height, width) * MAX_SIGN_RADIUS_FRACTION
    if radius < min_radius or radius > max_radius:
        return

    x1 = max(0, int(cx - radius))
    y1 = max(0, int(cy - radius))
    x2 = min(width, int(cx + radius))
    y2 = min(height, int(cy + radius))
    if x2 <= x1 or y2 <= y1:
        return

    white_ratio = bright_ratio(gray, (x1, y1, x2, y2))
    if white_ratio < MIN_SIGN_BRIGHT_RATIO:
        return

    for candidate in candidates:
        old_cx, old_cy, old_radius = candidate["circle"]
        center_dist = ((cx - old_cx) ** 2 + (cy - old_cy) ** 2) ** 0.5
        if center_dist < min(radius, old_radius) * 0.35:
            if white_ratio > candidate["white_ratio"]:
                candidate.update(
                    {
                        "circle": (float(cx), float(cy), float(radius)),
                        "box": (x1, y1, x2, y2),
                        "white_ratio": white_ratio,
                        "source": source,
                    }
                )
            return

    candidates.append(
        {
            "circle": (float(cx), float(cy), float(radius)),
            "box": (x1, y1, x2, y2),
            "white_ratio": white_ratio,
            "source": source,
        }
    )


def find_sign_candidates(gray, dark_mask):
    candidates = []
    height, width = gray.shape[:2]

    contours, _hierarchy = cv2.findContours(
        dark_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < height * width * 0.01:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue

        aspect = float(w) / float(h)
        if aspect < 0.65 or aspect > 1.35:
            continue

        circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
        if circularity < MIN_CIRCLE_CIRCULARITY:
            continue

        radius = 0.5 * max(w, h)
        add_sign_candidate(
            candidates,
            gray,
            x + w / 2.0,
            y + h / 2.0,
            radius,
            "contour",
        )

    blur = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(30, min(height, width) // 4),
        param1=90,
        param2=28,
        minRadius=int(min(height, width) * MIN_SIGN_RADIUS_FRACTION),
        maxRadius=int(min(height, width) * MAX_SIGN_RADIUS_FRACTION),
    )
    if circles is not None:
        for cx, cy, radius in np.round(circles[0, :]).astype(int):
            add_sign_candidate(candidates, gray, cx, cy, radius, "hough")

    candidates.sort(key=lambda item: item["white_ratio"], reverse=True)
    return candidates[:MAX_SIGN_CANDIDATES]


def inner_sign_box(candidate, shape):
    height, width = shape[:2]
    cx, cy, radius = candidate["circle"]
    inner_radius = radius * CIRCLE_INNER_SCALE
    x1 = max(0, int(cx - inner_radius))
    y1 = max(0, int(cy - inner_radius))
    x2 = min(width, int(cx + inner_radius))
    y2 = min(height, int(cy + inner_radius))
    return x1, y1, x2, y2


def find_arrow_component(mask, offset=(0, 0)):
    image_area = mask.shape[0] * mask.shape[1]
    min_box_size = int(min(mask.shape[:2]) * MIN_BOX_FRACTION)
    num_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
    offset_x, offset_y = offset

    best = None
    for label in range(1, num_labels):
        x, y, w, h, area = stats[label]
        if area < MIN_COMPONENT_AREA:
            continue
        if w < min_box_size or h < min_box_size:
            continue

        bbox_area = max(1, w * h)
        extent = float(area) / float(bbox_area)
        area_fraction = float(area) / float(image_area)
        if extent < MIN_COMPONENT_EXTENT:
            continue

        score = area_fraction * 4.0 + extent
        if best is None or score > best["score"]:
            component_mask = (labels[y : y + h, x : x + w] == label).astype(np.uint8)
            best = {
                "box": (
                    int(x + offset_x),
                    int(y + offset_y),
                    int(x + w + offset_x),
                    int(y + h + offset_y),
                ),
                "mask": component_mask,
                "area": int(area),
                "extent": extent,
                "score": score,
            }

    return best


def estimate_arrow_direction(component):
    component_mask = component["mask"]
    ys, xs = np.where(component_mask > 0)
    if len(xs) == 0:
        return None

    height = component_mask.shape[0]
    width = component_mask.shape[1]
    if width < 10 or height < 10:
        return None

    band_width = max(3, int(width * EDGE_BAND_FRACTION))
    top_limit = int(height * 0.65)
    top_pixels = ys <= top_limit

    left_top_pixels = ys[(xs <= band_width) & top_pixels]
    right_top_pixels = ys[(xs >= width - band_width - 1) & top_pixels]

    left_head_score = 0.0
    right_head_score = 0.0
    if len(left_top_pixels) > 0:
        left_span = int(left_top_pixels.max() - left_top_pixels.min() + 1)
        left_head_score = left_span + 0.03 * len(left_top_pixels)
    if len(right_top_pixels) > 0:
        right_span = int(right_top_pixels.max() - right_top_pixels.min() + 1)
        right_head_score = right_span + 0.03 * len(right_top_pixels)

    if left_head_score > right_head_score * MIN_HEAD_SCORE_RATIO:
        direction = "left"
    elif right_head_score > left_head_score * MIN_HEAD_SCORE_RATIO:
        direction = "right"
    else:
        return None

    max_head_score = max(left_head_score, right_head_score, 1.0)
    head_separation = abs(left_head_score - right_head_score) / max_head_score

    score = min(
        1.0,
        0.35
        + 0.45 * head_separation
        + 0.25 * min(1.0, component["extent"] / 0.45),
    )
    return direction, score


def detect_arrow(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = preprocess_dark_mask(img)
    sign_candidates = find_sign_candidates(gray, mask)
    if not sign_candidates:
        return None, mask

    best_detection = None
    for sign_candidate in sign_candidates:
        x1, y1, x2, y2 = inner_sign_box(sign_candidate, mask.shape)
        sign_mask = mask[y1:y2, x1:x2]
        component = find_arrow_component(sign_mask, offset=(x1, y1))
        if component is None:
            continue

        direction_score = estimate_arrow_direction(component)
        if direction_score is None:
            continue

        direction, score = direction_score
        sign_bonus = min(0.15, sign_candidate["white_ratio"] * 0.15)
        score = min(1.0, score + sign_bonus)
        bx1, by1, bx2, by2 = component["box"]
        detection = {
            "direction": direction,
            "confidence": score,
            "box": (bx1, by1, bx2, by2),
            "sign_box": sign_candidate["box"],
            "sign_circle": sign_candidate["circle"],
            "area": component["area"],
            "extent": component["extent"],
            "source": sign_candidate["source"],
        }
        if best_detection is None or score > best_detection["confidence"]:
            best_detection = detection

    return best_detection, mask


def draw_qr(img, qr_text, qr_points):
    if qr_points is not None:
        points = qr_points.astype(int).reshape(-1, 2)
        for i in range(len(points)):
            cv2.line(
                img,
                tuple(points[i]),
                tuple(points[(i + 1) % len(points)]),
                (255, 0, 255),
                2,
            )
    if qr_text:
        cv2.putText(
            img,
            f"QR: {qr_text[:40]}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )


def draw_arrow(img, detection):
    if detection is None:
        return

    sx1, sy1, sx2, sy2 = detection["sign_box"]
    cx, cy, radius = detection["sign_circle"]
    x1, y1, x2, y2 = detection["box"]
    direction = detection["direction"]
    confidence = detection["confidence"]
    color = (0, 200, 255) if direction == "left" else (0, 255, 0)
    label = f"arrow {direction} {confidence:.2f}"

    cv2.rectangle(img, (sx1, sy1), (sx2, sy2), (255, 255, 0), 1)
    cv2.circle(img, (int(cx), int(cy)), int(radius), (255, 255, 0), 1)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        img,
        label,
        (x1, max(22, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


def update_arrow_state(detection, qr_text, source_frame_time):
    global _candidate_direction, _candidate_count, _stable_direction

    if detection is None or detection["confidence"] < ARROW_SCORE_THRESHOLD:
        old_stable_direction = _stable_direction
        _candidate_direction = "none"
        _candidate_count = 0
        _stable_direction = "none"
        if qr_text:
            write_latest_signal("qr", "none", qr_text, 1.0, source_frame_time)
        elif old_stable_direction != "none":
            write_latest_signal("none", "none", None, 0.0, source_frame_time)
            print("[HANDCRAFT] Flecha perdida. Estado: none")
        return

    direction = detection["direction"]
    if direction == _candidate_direction:
        _candidate_count += 1
    else:
        _candidate_direction = direction
        _candidate_count = 1

    if _candidate_count >= ARROW_STABLE_FRAMES and direction != _stable_direction:
        _stable_direction = direction
        write_latest_signal(
            "arrow",
            direction,
            qr_text or None,
            detection["confidence"],
            source_frame_time,
        )
        print(
            f"[HANDCRAFT] Flecha estable: {direction} "
            f"score={detection['confidence']:.2f} frame_time={source_frame_time}"
        )


def process_vision_from_image(img, sec, nsec):
    global _vision_frame_counter, _last_qr_text

    _vision_frame_counter += 1
    source_frame_time = f"{sec}.{nsec:09d}"
    if _vision_frame_counter % VISION_CHECK_EVERY_N_FRAMES != 0:
        return

    qr_text, qr_points, _ = _qr_detector.detectAndDecode(img)
    draw_qr(img, qr_text, qr_points)

    if qr_text and qr_text != _last_qr_text:
        print(f"[QR] Checkpoint detectado: {qr_text}")
    _last_qr_text = qr_text or None
    if qr_text:
        update_arrow_state(None, qr_text, source_frame_time)
        return

    arrow_detection, _mask = detect_arrow(img)
    draw_arrow(img, arrow_detection)
    update_arrow_state(arrow_detection, qr_text or None, source_frame_time)


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

        process_vision_from_image(img, sec, nsec)

        cv2.imshow(f"Handcraft {robot_name} (domain {domain_id})", img)
        cv2.waitKey(1)
    except Exception as e:
        print(f"[IMG] Error: {e}")


def main():
    write_latest_signal("none", "none", None, 0.0, None)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (ROBOT_IP, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    print("[MAIN] Recibiendo telemetria handcraft. Ctrl+C para salir.")
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
        write_latest_signal("none", "none", None, 0.0, None)
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
