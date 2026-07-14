#!/usr/bin/env python3
"""Robust QR decoding helpers for robot camera frames."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class QRDecodeResult:
    content: Optional[str]
    status: str
    variant: str
    detected_count: int = 0
    error: Optional[str] = None


def _first_payload(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip()
        return text or None
    if isinstance(value, (list, tuple)):
        for item in value:
            payload = _first_payload(item)
            if payload:
                return payload
    return None


def _variants(cv2, image) -> Iterable[tuple[str, Any]]:
    """Yield cheap QR-friendly image variants without changing dependencies."""

    yield "bgr", image
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    except Exception:
        gray = image
    yield "gray", gray

    try:
        up_gray = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        yield "gray_2x", up_gray
    except Exception:
        up_gray = None

    try:
        equalized = cv2.equalizeHist(gray)
        yield "gray_equalized", equalized
        yield "gray_equalized_2x", cv2.resize(equalized, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    except Exception:
        pass

    try:
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        thresh = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            2,
        )
        yield "adaptive_threshold", thresh
        yield "adaptive_threshold_2x", cv2.resize(thresh, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_NEAREST)
    except Exception:
        pass

    if up_gray is None:
        return


def decode_qr_image(detector, image) -> QRDecodeResult:
    """Decode one QR payload from an OpenCV image using several stable variants."""

    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - robot dependency
        return QRDecodeResult(None, "cv2_unavailable", "none", error=str(exc))

    detected_count = 0
    last_error = None
    for variant_name, variant in _variants(cv2, image):
        try:
            if hasattr(detector, "detectAndDecodeMulti"):
                ok, decoded_info, points, _ = detector.detectAndDecodeMulti(variant)
                if ok:
                    try:
                        detected_count = max(detected_count, len(decoded_info or []))
                    except TypeError:
                        detected_count = max(detected_count, 1)
                    payload = _first_payload(decoded_info)
                    if payload:
                        return QRDecodeResult(payload, "decoded", variant_name, detected_count)

            decoded, points, _ = detector.detectAndDecode(variant)
            if points is not None:
                detected_count = max(detected_count, 1)
            payload = _first_payload(decoded)
            if payload:
                return QRDecodeResult(payload, "decoded", variant_name, detected_count)

            detected, points = detector.detect(variant)
            if detected:
                detected_count = max(detected_count, 1)
                decoded = detector.decode(variant, points)
                payload = _first_payload(decoded)
                if payload:
                    return QRDecodeResult(payload, "decoded", f"{variant_name}:detect_decode", detected_count)
        except Exception as exc:
            last_error = str(exc)
            continue

    if detected_count > 0:
        return QRDecodeResult(None, "detected_not_decoded", "none", detected_count, last_error)
    return QRDecodeResult(None, "not_detected", "none", detected_count, last_error)
