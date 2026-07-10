#!/usr/bin/env python3
"""Compara OpenCV QRCodeDetector y pyzbar usando una webcam."""

import sys

import cv2

try:
    from pyzbar import pyzbar
except ImportError as exc:
    raise SystemExit(
        "Falta pyzbar. Instala las dependencias con: uv add pyzbar"
    ) from exc


def main() -> None:
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise SystemExit(
            f"No se pudo abrir la webcam {camera_index}. "
            "Prueba con el indice 0: uv run python win/zbar/compare_qr_detectors.py 0"
        )

    print("Comparando detectores QR. Presiona 'q' para salir.")
    cv2_detector = cv2.QRCodeDetector()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("No se pudo capturar un frame.")
                break

            frame_cv2 = frame.copy()
            frame_pyzbar = frame.copy()

            value, points, _ = cv2_detector.detectAndDecode(frame_cv2)
            if points is not None:
                points = points.astype(int).reshape(-1, 2)
                for i, point in enumerate(points):
                    cv2.line(
                        frame_cv2,
                        tuple(point),
                        tuple(points[(i + 1) % len(points)]),
                        (0, 0, 255),
                        3,
                    )
                label = value or "Detectado, pero no decodificado"
            else:
                label = "No se detecto QR"
            cv2.putText(
                frame_cv2, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255), 2,
            )

            decoded_objects = pyzbar.decode(frame_pyzbar)
            if decoded_objects:
                for obj in decoded_objects:
                    polygon = [(point.x, point.y) for point in obj.polygon]
                    if len(polygon) >= 4:
                        for i, point in enumerate(polygon):
                            cv2.line(
                                frame_pyzbar,
                                point,
                                polygon[(i + 1) % len(polygon)],
                                (0, 255, 0),
                                3,
                            )
                    else:
                        x, y, width, height = obj.rect
                        cv2.rectangle(
                            frame_pyzbar,
                            (x, y),
                            (x + width, y + height),
                            (0, 255, 0),
                            3,
                        )
                    text = obj.data.decode("utf-8", errors="replace")
                    cv2.putText(
                        frame_pyzbar, text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                    )
            else:
                cv2.putText(
                    frame_pyzbar, "No se detecto QR", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )

            cv2.imshow("OpenCV QRCodeDetector (rojo)", frame_cv2)
            cv2.imshow("pyzbar (verde)", frame_pyzbar)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
