#!/usr/bin/env python3
"""Bounded ZXing QR decoder for laptop-side camera frames."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Iterable, Optional

import cv2
import numpy as np


DEFAULT_VARIANTS = (
    "original",
    "gray",
    "clahe",
    "gray_2x",
    "gray_3x",
    "clahe_2x",
    "sharpen",
    "center_80",
    "inverted_gray",
)


@dataclass(frozen=True)
class QRDecodeCandidate:
    raw_payload: Optional[str]
    status: str
    barcode_format: Optional[str] = None
    variant: str = "none"
    corners: tuple[tuple[int, int], ...] = ()
    decode_latency_ms: float = 0.0
    error: Optional[str] = None


def _variants(image: np.ndarray, enabled: Iterable[str]) -> Iterable[tuple[str, np.ndarray]]:
    enabled_set = set(enabled)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    if "original" in enabled_set:
        yield "original", image
    if "gray" in enabled_set:
        yield "gray", gray
    if "clahe" in enabled_set:
        yield "clahe", clahe
    if "gray_2x" in enabled_set:
        yield "gray_2x", cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    if "gray_3x" in enabled_set:
        yield "gray_3x", cv2.resize(gray, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
    if "clahe_2x" in enabled_set:
        yield "clahe_2x", cv2.resize(clahe, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
    if "sharpen" in enabled_set:
        blurred = cv2.GaussianBlur(gray, (0, 0), 1.0)
        yield "sharpen", cv2.addWeighted(gray, 1.7, blurred, -0.7, 0)
    if "center_80" in enabled_set:
        height, width = gray.shape[:2]
        margin_x, margin_y = int(width * 0.10), int(height * 0.10)
        yield "center_80", gray[margin_y : height - margin_y, margin_x : width - margin_x]
    if "inverted_gray" in enabled_set:
        yield "inverted_gray", cv2.bitwise_not(gray)


def _corners(position: Any) -> tuple[tuple[int, int], ...]:
    if position is None:
        return ()
    values = []
    for name in ("top_left", "top_right", "bottom_right", "bottom_left"):
        point = getattr(position, name, None)
        if point is not None:
            values.append((int(point.x), int(point.y)))
    return tuple(values)


class ZXingQRDecoder:
    def __init__(self, enabled_variants: Iterable[str] = DEFAULT_VARIANTS):
        self.enabled_variants = tuple(enabled_variants)
        self.available = True
        self.import_error: Optional[str] = None
        try:
            import zxingcpp
        except ImportError as exc:
            self.available = False
            self.import_error = str(exc)
            self._zxing = None
        else:
            self._zxing = zxingcpp

    def decode(self, image: np.ndarray) -> QRDecodeCandidate:
        started = time.perf_counter()
        if not self.available or self._zxing is None:
            return QRDecodeCandidate(
                None,
                "decoder_unavailable",
                decode_latency_ms=(time.perf_counter() - started) * 1000.0,
                error=self.import_error,
            )
        last_error = None
        try:
            variants = _variants(image, self.enabled_variants)
            for variant_name, variant in variants:
                try:
                    results = self._zxing.read_barcodes(
                        variant,
                        formats=self._zxing.BarcodeFormat.QRCode,
                        try_rotate=False,
                        try_downscale=True,
                        try_invert=False,
                    )
                except Exception as exc:
                    last_error = str(exc)
                    continue
                for result in results:
                    raw = str(getattr(result, "text", "") or "")
                    if raw:
                        return QRDecodeCandidate(
                            raw,
                            "decoded_candidate",
                            barcode_format=str(getattr(result, "format", "QRCode")),
                            variant=variant_name,
                            corners=_corners(getattr(result, "position", None)),
                            decode_latency_ms=(time.perf_counter() - started) * 1000.0,
                        )
        except Exception as exc:
            return QRDecodeCandidate(
                None,
                "decoder_error",
                decode_latency_ms=(time.perf_counter() - started) * 1000.0,
                error=str(exc),
            )
        return QRDecodeCandidate(
            None,
            "no_candidate" if last_error is None else "decoder_error",
            decode_latency_ms=(time.perf_counter() - started) * 1000.0,
            error=last_error,
        )
