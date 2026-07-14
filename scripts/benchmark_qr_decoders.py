#!/usr/bin/env python3
"""Benchmark OpenCV and laptop ZXing QR decoders on one labeled manifest."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
import sys
import time
from typing import Any, Iterable

import cv2

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ubuntu.reactive_nav.qr_detection import decode_qr_image
from win.yolo.qr_validator import normalize_qr_payload
from win.yolo.qr_zxing import DEFAULT_VARIANTS, ZXingQRDecoder


DECODERS = ("opencv_raw", "opencv_cascade", "zxing")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", nargs="+", help="One or more JSONL manifests from scripts/capture_qr_dataset.py.")
    parser.add_argument("--out-dir", default=str(REPO_ROOT / "output" / "qr_zxing_benchmark"))
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--zxing-variants", default=",".join(DEFAULT_VARIANTS))
    return parser.parse_args()


def read_manifest(path: Path) -> list[dict[str, Any]]:
    records = []
    base = path.parent
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        record = json.loads(line)
        image_path = Path(record.get("path") or "")
        if not image_path.is_absolute():
            image_path = base / image_path
        if not image_path.exists() and record.get("relative_path"):
            image_path = base / str(record["relative_path"])
        record["path"] = str(image_path)
        record["manifest_line"] = line_number
        records.append(record)
    return records


def normalize_expected(value: Any) -> str:
    normalized, _reason = normalize_qr_payload(value)
    return normalized or ""


def classify(valid: bool, expected: str, decoded: str | None) -> tuple[bool, bool, str]:
    decoded_norm, _reason = normalize_qr_payload(decoded)
    decoded_norm = decoded_norm or ""
    if valid:
        ok = bool(decoded_norm) and (not expected or decoded_norm == expected)
        return ok, False, "true_positive" if ok else "miss"
    if decoded_norm:
        return False, True, "false_positive"
    return True, False, "true_negative"


def decode_opencv_raw(detector: Any, image: Any) -> tuple[str | None, str, float]:
    started = time.perf_counter()
    try:
        payload, points, _ = detector.detectAndDecode(image)
        status = "decoded" if payload else ("detected_not_decoded" if points is not None else "not_detected")
        return payload or None, status, (time.perf_counter() - started) * 1000.0
    except Exception as exc:
        return None, f"error:{exc}", (time.perf_counter() - started) * 1000.0


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    for decoder in DECODERS:
        subset = [row for row in rows if row["decoder"] == decoder]
        latencies = [float(row["latency_ms"]) for row in subset]
        positives = [row for row in subset if row["valid"]]
        negatives = [row for row in subset if not row["valid"]]
        true_positive = sum(1 for row in positives if row["ok"])
        false_positive = sum(1 for row in negatives if row["false_positive"])
        recall = true_positive / len(positives) if positives else None
        fp_rate = false_positive / len(negatives) if negatives else 0.0
        summary.append(
            {
                "decoder": decoder,
                "samples": len(subset),
                "positive_samples": len(positives),
                "negative_samples": len(negatives),
                "true_positive": true_positive,
                "false_positive": false_positive,
                "recall": recall,
                "false_positive_rate": fp_rate,
                "median_latency_ms": statistics.median(latencies) if latencies else None,
                "p95_latency_ms": percentile(latencies, 95) if latencies else None,
            }
        )
    return summary


def percentile(values: Iterable[float], pct: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct / 100.0
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_summary_md(path: Path, summary: list[dict[str, Any]]) -> None:
    lines = [
        "# QR Decoder Benchmark",
        "",
        "| decoder | samples | recall | false positives | median ms | p95 ms |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        recall = "n/a" if row["recall"] is None else f"{row['recall']:.3f}"
        median = "n/a" if row["median_latency_ms"] is None else f"{row['median_latency_ms']:.1f}"
        p95 = "n/a" if row["p95_latency_ms"] is None else f"{row['p95_latency_ms']:.1f}"
        lines.append(
            f"| {row['decoder']} | {row['samples']} | {recall} | {row['false_positive']} | {median} | {p95} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    manifest_paths = [Path(item).expanduser().resolve() for item in args.manifest]
    records: list[dict[str, Any]] = []
    for manifest_path in manifest_paths:
        for record in read_manifest(manifest_path):
            record["manifest"] = str(manifest_path)
            records.append(record)
    run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir).expanduser().resolve() / run_id
    out_dir.mkdir(parents=True, exist_ok=True)
    variants = tuple(item.strip() for item in args.zxing_variants.split(",") if item.strip())
    decoder = ZXingQRDecoder(variants)
    detector = cv2.QRCodeDetector()

    rows: list[dict[str, Any]] = []
    ablation_rows: list[dict[str, Any]] = []
    for sample_index, record in enumerate(records, start=1):
        image = cv2.imread(record["path"])
        expected = normalize_expected(record.get("expected_payload"))
        valid = bool(record.get("valid", True))
        if image is None:
            raise SystemExit(f"failed to read image: {record['path']}")

        raw_payload, raw_status, raw_latency = decode_opencv_raw(detector, image)
        cascade_started = time.perf_counter()
        cascade = decode_qr_image(detector, image)
        cascade_latency = (time.perf_counter() - cascade_started) * 1000.0
        zxing = decoder.decode(image)

        results = {
            "opencv_raw": (raw_payload, raw_status, raw_latency, "raw"),
            "opencv_cascade": (cascade.content, cascade.status, cascade_latency, cascade.variant),
            "zxing": (zxing.raw_payload, zxing.status, zxing.decode_latency_ms, zxing.variant),
        }
        for decoder_name, (payload, status, latency, variant) in results.items():
            ok, false_positive, outcome = classify(valid, expected, payload)
            rows.append(
                {
                    "sample_index": sample_index,
                    "path": record["path"],
                    "bucket": record.get("bucket", ""),
                    "valid": valid,
                    "expected_payload": expected,
                    "decoder": decoder_name,
                    "decoded_payload": payload or "",
                    "status": status,
                    "variant": variant,
                    "latency_ms": round(float(latency), 3),
                    "ok": ok,
                    "false_positive": false_positive,
                    "outcome": outcome,
                }
            )

        for variant in variants:
            variant_result = ZXingQRDecoder((variant,)).decode(image)
            ok, false_positive, outcome = classify(valid, expected, variant_result.raw_payload)
            ablation_rows.append(
                {
                    "sample_index": sample_index,
                    "path": record["path"],
                    "bucket": record.get("bucket", ""),
                    "valid": valid,
                    "expected_payload": expected,
                    "variant": variant,
                    "decoded_payload": variant_result.raw_payload or "",
                    "status": variant_result.status,
                    "latency_ms": round(float(variant_result.decode_latency_ms), 3),
                    "ok": ok,
                    "false_positive": false_positive,
                    "outcome": outcome,
                }
            )

    summary = summarize(rows)
    config = {
        "manifests": [str(path) for path in manifest_paths],
        "run_id": run_id,
        "zxing_available": decoder.available,
        "zxing_import_error": decoder.import_error,
        "zxing_variants": variants,
        "sample_count": len(records),
    }
    (out_dir / "config.json").write_text(json.dumps(config, indent=2, ensure_ascii=True), encoding="utf-8")
    with (out_dir / "per_sample.jsonl").open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=True) + "\n")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=True), encoding="utf-8")
    write_csv(out_dir / "summary.csv", summary)
    write_summary_md(out_dir / "summary.md", summary)
    write_csv(out_dir / "stage_ablation.csv", ablation_rows)
    (out_dir / "yolo_regression.json").write_text(
        json.dumps(
            {
                "status": "not_run",
                "reason": "This offline QR benchmark does not load the YOLO model. Use live perception logs to compare YOLO timing with QR disabled/enabled.",
            },
            indent=2,
            ensure_ascii=True,
        ),
        encoding="utf-8",
    )
    print(f"[BENCH] wrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
