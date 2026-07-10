#!/usr/bin/env python3
"""Evaluate YOLO signal detections and FSM acceptance on labels-gt images."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault("YOLO_CONFIG_DIR", str(REPO_ROOT / "output" / "ultralytics_config"))
os.environ.setdefault("ULTRALYTICS_CONFIG_DIR", str(REPO_ROOT / "output" / "ultralytics_config"))
os.environ.setdefault("MPLCONFIGDIR", str(REPO_ROOT / "output" / "matplotlib_config"))
for _config_dir in (
    Path(os.environ["YOLO_CONFIG_DIR"]),
    Path(os.environ["ULTRALYTICS_CONFIG_DIR"]),
    Path(os.environ["MPLCONFIGDIR"]),
):
    _config_dir.mkdir(parents=True, exist_ok=True)

from scripts.replay_nav_scenarios import build_arbiter, corridor_scan, load_replay_profile
from ubuntu.reactive_nav.behavior_arbiter import ArbiterInput, SignalState
from ubuntu.reactive_nav.lidar_sectors import extract_sectors
from ubuntu.reactive_nav.wall_following import NavigationSuggestion, TwistCommand
from win.yolo.recibidor import (
    ACTION_CENTER_X_MAX,
    ACTION_CENTER_X_MIN,
    ACTION_CONF_THRESHOLD,
    ACTION_MIN_AREA_RATIO,
    CONF_THRESHOLD,
    IMG_SIZE,
    STABLE_SIGNAL_FRAMES,
    signal_direction_from_name,
)


CLASS_TO_DIRECTION = {
    0: "left",
    1: "right",
    2: "stop",
    3: "none",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset",
        default="labels-gt/dataset",
        help="Dataset root with images/<split> and labels/<split>.",
    )
    parser.add_argument(
        "--split",
        choices=("train", "val", "all"),
        default="all",
        help="Dataset split to evaluate.",
    )
    parser.add_argument(
        "--config",
        default="ubuntu/reactive_nav/configs/wall_follow_less_conservative_1.yaml",
        help="Reactive navigation config to use for robot-side sign gates and FSM.",
    )
    parser.add_argument("--model", default="models/signals/best.pt", help="YOLO model path.")
    parser.add_argument("--out-dir", default=None, help="Output directory. Defaults to output/signal_fsm_eval/<run_id>.")
    parser.add_argument("--limit", type=int, default=0, help="Optional max images for quick checks.")
    parser.add_argument("--conf", type=float, default=CONF_THRESHOLD, help="YOLO prediction confidence threshold.")
    parser.add_argument("--imgsz", type=int, default=IMG_SIZE, help="YOLO image size.")
    return parser.parse_args()


def _splits(split: str) -> Sequence[str]:
    return ("train", "val") if split == "all" else (split,)


def image_paths(dataset: Path, split: str, limit: int) -> List[Path]:
    paths: List[Path] = []
    for name in _splits(split):
        paths.extend(sorted((dataset / "images" / name).glob("*.jpg")))
    if limit > 0:
        paths = paths[:limit]
    return paths


def label_path_for(dataset: Path, image_path: Path) -> Path:
    split = image_path.parent.name
    return dataset / "labels" / split / f"{image_path.stem}.txt"


def read_labels(path: Path) -> List[Dict[str, float | int | str]]:
    labels: List[Dict[str, float | int | str]] = []
    if not path.exists():
        return labels
    for raw in path.read_text(encoding="utf-8").splitlines():
        parts = raw.split()
        if len(parts) < 5:
            continue
        class_id = int(float(parts[0]))
        x_center, y_center, width, height = (float(value) for value in parts[1:5])
        labels.append(
            {
                "class_id": class_id,
                "direction": CLASS_TO_DIRECTION.get(class_id, "none"),
                "x_center": x_center,
                "y_center": y_center,
                "width": width,
                "height": height,
                "area_ratio": width * height,
            }
        )
    return labels


def expected_direction(labels: Sequence[Dict[str, Any]]) -> str:
    signal_labels = [item for item in labels if item.get("direction") in ("left", "right", "stop")]
    if not signal_labels:
        return "none"
    largest = max(signal_labels, key=lambda item: float(item.get("area_ratio") or 0.0))
    return str(largest.get("direction") or "none")


def gt_xyxy(label: Dict[str, Any], width: int, height: int) -> Tuple[float, float, float, float]:
    cx = float(label["x_center"]) * width
    cy = float(label["y_center"]) * height
    box_w = float(label["width"]) * width
    box_h = float(label["height"]) * height
    return (cx - box_w / 2.0, cy - box_h / 2.0, cx + box_w / 2.0, cy + box_h / 2.0)


def iou(box_a: Sequence[float], box_b: Sequence[float]) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter = inter_w * inter_h
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter
    return inter / denom if denom > 0.0 else 0.0


def best_signal_detection(result: Any) -> Optional[Dict[str, Any]]:
    if result.boxes is None or len(result.boxes) == 0:
        return None
    height, width = result.orig_shape[:2]
    names = result.names
    best: Optional[Dict[str, Any]] = None
    xyxy = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    for box, conf, class_id in zip(xyxy, confs, classes):
        name = names.get(int(class_id), f"class_{int(class_id)}")
        direction = signal_direction_from_name(str(name))
        if direction is None:
            continue
        x1, y1, x2, y2 = [float(value) for value in box]
        x1, x2 = max(0.0, min(width - 1, x1)), max(0.0, min(width - 1, x2))
        y1, y2 = max(0.0, min(height - 1, y1)), max(0.0, min(height - 1, y2))
        area_ratio = max(0.0, x2 - x1) * max(0.0, y2 - y1) / max(1, width * height)
        center_x_ratio = ((x1 + x2) / 2.0) / max(1, width)
        actionable = (
            float(conf) >= ACTION_CONF_THRESHOLD
            and area_ratio >= ACTION_MIN_AREA_RATIO
            and ACTION_CENTER_X_MIN <= center_x_ratio <= ACTION_CENTER_X_MAX
        )
        candidate = {
            "direction": direction,
            "confidence": float(conf),
            "class_id": int(class_id),
            "class_name": str(name),
            "bbox_xyxy": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
            "bbox_area_ratio": area_ratio,
            "bbox_center_x_ratio": center_x_ratio,
            "actionable": actionable,
        }
        if best is None or candidate["confidence"] > best["confidence"]:
            best = candidate
    return best


def best_iou_for_detection(
    detection: Optional[Dict[str, Any]],
    labels: Sequence[Dict[str, Any]],
    width: int,
    height: int,
) -> float:
    if detection is None:
        return 0.0
    matching = [label for label in labels if label.get("direction") == detection.get("direction")]
    if not matching:
        return 0.0
    pred_box = [float(value) for value in detection["bbox_xyxy"]]
    return max(iou(pred_box, gt_xyxy(label, width, height)) for label in matching)


def safe_sectors():
    return extract_sectors(
        corridor_scan(front=1.8, front_center=1.8, front_left=1.4, front_right=1.4, left=0.8, right=0.8)
    )


def fsm_response(
    detection: Optional[Dict[str, Any]],
    params: Dict[str, Any],
    *,
    require_laptop_actionable: bool,
) -> Dict[str, Any]:
    arbiter = build_arbiter(params)
    sectors = safe_sectors()
    nav = NavigationSuggestion(TwistCommand(0.12, 0.0), "CORRIDOR_FOLLOW", "DATASET_SIGNAL_FSM_EVAL")
    repeat_count = max(int(params.get("sign_confirm_count", 2)), STABLE_SIGNAL_FRAMES)
    now = 100.0
    output = None
    if detection is None:
        signal = SignalState(timestamp=now, stale=False, reason="dataset_no_actionable_signal")
        output = arbiter.decide(ArbiterInput(sectors, True, nav, signal, False, now))
    else:
        robot_gate_pass = (
            float(detection["confidence"]) >= float(params["sign_min_confidence"])
            and float(detection["bbox_area_ratio"]) >= float(params["sign_min_area_ratio"])
        )
        signal_actionable = bool(detection.get("actionable")) if require_laptop_actionable else robot_gate_pass
        if not signal_actionable:
            signal = SignalState(
                direction=str(detection["direction"]),
                confidence=float(detection["confidence"]),
                bbox_area_ratio=float(detection["bbox_area_ratio"]),
                bbox_center_x_ratio=float(detection["bbox_center_x_ratio"]),
                actionable=False,
                timestamp=now,
                stale=False,
                event_id=f"dataset:{detection['direction']}:{detection['bbox_xyxy']}",
                reason="dataset_below_action_gate",
                raw_class=str(detection["class_name"]),
            )
            output = arbiter.decide(ArbiterInput(sectors, True, nav, signal, False, now))
            assert output is not None
            return {
                "state": output.state,
                "reason": output.reason,
                "command_linear_x": output.command.linear_x,
                "command_angular_z": output.command.angular_z,
                "command_source": output.debug.get("command_source"),
                "yolo_event": output.debug.get("yolo_event"),
                "yolo_event_status": output.debug.get("yolo_event_status"),
                "yolo_rejection_reason": output.debug.get("yolo_rejection_reason"),
                "yolo_confirmation_progress": output.debug.get("yolo_confirmation_progress"),
            }
        for index in range(repeat_count):
            signal = SignalState(
                direction=str(detection["direction"]),
                confidence=float(detection["confidence"]),
                bbox_area_ratio=float(detection["bbox_area_ratio"]),
                bbox_center_x_ratio=float(detection["bbox_center_x_ratio"]),
                actionable=signal_actionable,
                timestamp=now + index * 0.1,
                stale=False,
                event_id=f"dataset:{detection['direction']}:{detection['bbox_xyxy']}",
                reason="dataset_fresh",
                raw_class=str(detection["class_name"]),
            )
            output = arbiter.decide(ArbiterInput(sectors, True, nav, signal, False, now + index * 0.1))
    assert output is not None
    return {
        "state": output.state,
        "reason": output.reason,
        "command_linear_x": output.command.linear_x,
        "command_angular_z": output.command.angular_z,
        "command_source": output.debug.get("command_source"),
        "yolo_event": output.debug.get("yolo_event"),
        "yolo_event_status": output.debug.get("yolo_event_status"),
        "yolo_rejection_reason": output.debug.get("yolo_rejection_reason"),
        "yolo_confirmation_progress": output.debug.get("yolo_confirmation_progress"),
    }


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def write_jsonl(path: Path, rows: Iterable[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    fields = [
        "image",
        "split",
        "expected_direction",
        "pred_direction",
        "pred_confidence",
        "pred_area_ratio",
        "pred_center_x_ratio",
        "pred_actionable",
        "direction_correct",
        "iou",
        "autonomous_fsm_state",
        "autonomous_fsm_reason",
        "autonomous_fsm_yolo_status",
        "autonomous_fsm_rejection_reason",
        "robot_gate_fsm_state",
        "robot_gate_fsm_yolo_status",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            detection = row.get("detection") or {}
            autonomous_fsm = row.get("autonomous_fsm") or {}
            robot_gate_fsm = row.get("robot_gate_fsm") or {}
            writer.writerow(
                {
                    "image": row["image"],
                    "split": row["split"],
                    "expected_direction": row["expected_direction"],
                    "pred_direction": detection.get("direction") or "none",
                    "pred_confidence": detection.get("confidence"),
                    "pred_area_ratio": detection.get("bbox_area_ratio"),
                    "pred_center_x_ratio": detection.get("bbox_center_x_ratio"),
                    "pred_actionable": detection.get("actionable", False),
                    "direction_correct": row["direction_correct"],
                    "iou": row["iou"],
                    "autonomous_fsm_state": autonomous_fsm.get("state"),
                    "autonomous_fsm_reason": autonomous_fsm.get("reason"),
                    "autonomous_fsm_yolo_status": autonomous_fsm.get("yolo_event_status"),
                    "autonomous_fsm_rejection_reason": autonomous_fsm.get("yolo_rejection_reason"),
                    "robot_gate_fsm_state": robot_gate_fsm.get("state"),
                    "robot_gate_fsm_yolo_status": robot_gate_fsm.get("yolo_event_status"),
                }
            )


def summarize(rows: Sequence[Dict[str, Any]], params: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    expected_counts = Counter(str(row["expected_direction"]) for row in rows)
    pred_counts = Counter(str((row.get("detection") or {}).get("direction") or "none") for row in rows)
    fsm_states = Counter(str(row["autonomous_fsm"].get("state")) for row in rows)
    fsm_status = Counter(str(row["autonomous_fsm"].get("yolo_event_status")) for row in rows)
    fsm_rejections = Counter(str(row["autonomous_fsm"].get("yolo_rejection_reason")) for row in rows)
    robot_gate_fsm_states = Counter(str(row["robot_gate_fsm"].get("state")) for row in rows)
    robot_gate_fsm_status = Counter(str(row["robot_gate_fsm"].get("yolo_event_status")) for row in rows)
    signal_rows = [row for row in rows if row["expected_direction"] in ("left", "right", "stop")]
    correct_direction = [row for row in signal_rows if row["direction_correct"]]
    iou50 = [row for row in correct_direction if float(row["iou"]) >= 0.50]
    robot_gate_pass = [
        row
        for row in correct_direction
        if (row.get("detection") or {}).get("confidence", 0.0) >= float(params["sign_min_confidence"])
        and (row.get("detection") or {}).get("bbox_area_ratio", 0.0) >= float(params["sign_min_area_ratio"])
    ]
    laptop_actionable = [row for row in correct_direction if (row.get("detection") or {}).get("actionable")]
    fsm_accepted = [row for row in signal_rows if row["autonomous_fsm"].get("yolo_event_status") == "accepted"]
    robot_gate_fsm_accepted = [
        row for row in signal_rows if row["robot_gate_fsm"].get("yolo_event_status") == "accepted"
    ]
    false_actionable = [
        row
        for row in rows
        if row["expected_direction"] == "none" and (row.get("detection") or {}).get("actionable")
    ]
    by_expected: Dict[str, Dict[str, Any]] = {}
    for direction in ("left", "right", "stop", "none"):
        subset = [row for row in rows if row["expected_direction"] == direction]
        if not subset:
            continue
        by_expected[direction] = {
            "images": len(subset),
            "correct_direction": sum(1 for row in subset if row["direction_correct"]),
            "laptop_actionable_correct": sum(
                1 for row in subset if row["direction_correct"] and (row.get("detection") or {}).get("actionable")
            ),
            "autonomous_fsm_accepted": sum(
                1 for row in subset if row["autonomous_fsm"].get("yolo_event_status") == "accepted"
            ),
            "robot_gate_fsm_accepted": sum(
                1 for row in subset if row["robot_gate_fsm"].get("yolo_event_status") == "accepted"
            ),
            "predicted": dict(Counter(str((row.get("detection") or {}).get("direction") or "none") for row in subset)),
        }
    return {
        "dataset": args.dataset,
        "split": args.split,
        "model": args.model,
        "config": args.config,
        "image_count": len(rows),
        "signal_image_count": len(signal_rows),
        "expected_counts": dict(expected_counts),
        "predicted_counts": dict(pred_counts),
        "direction_recall": len(correct_direction) / len(signal_rows) if signal_rows else 0.0,
        "iou50_recall": len(iou50) / len(signal_rows) if signal_rows else 0.0,
        "robot_gate_correct_recall": len(robot_gate_pass) / len(signal_rows) if signal_rows else 0.0,
        "laptop_actionable_correct_recall": len(laptop_actionable) / len(signal_rows) if signal_rows else 0.0,
        "fsm_accepted_recall": len(fsm_accepted) / len(signal_rows) if signal_rows else 0.0,
        "robot_gate_fsm_accepted_recall": (
            len(robot_gate_fsm_accepted) / len(signal_rows) if signal_rows else 0.0
        ),
        "false_actionable_on_non_signal": len(false_actionable),
        "fsm_states": dict(fsm_states),
        "fsm_yolo_status": dict(fsm_status),
        "fsm_rejection_reasons": dict(fsm_rejections),
        "robot_gate_fsm_states": dict(robot_gate_fsm_states),
        "robot_gate_fsm_yolo_status": dict(robot_gate_fsm_status),
        "by_expected": by_expected,
        "thresholds": {
            "model_conf": args.conf,
            "laptop_action_confidence": ACTION_CONF_THRESHOLD,
            "laptop_action_min_area_ratio": ACTION_MIN_AREA_RATIO,
            "laptop_center_x_min": ACTION_CENTER_X_MIN,
            "laptop_center_x_max": ACTION_CENTER_X_MAX,
            "laptop_stable_signal_frames": STABLE_SIGNAL_FRAMES,
            "robot_sign_min_confidence": params["sign_min_confidence"],
            "robot_sign_min_area_ratio": params["sign_min_area_ratio"],
            "robot_sign_confirm_count": params["sign_confirm_count"],
            "robot_sign_confirm_window": params["sign_confirm_window"],
        },
    }


def write_markdown(path: Path, summary: Dict[str, Any]) -> None:
    lines = [
        "# Signal/FSM Dataset Evaluation",
        "",
        f"- Dataset: `{summary['dataset']}`",
        f"- Split: `{summary['split']}`",
        f"- Model: `{summary['model']}`",
        f"- Config: `{summary['config']}`",
        f"- Images: {summary['image_count']} ({summary['signal_image_count']} signal images)",
        "",
        "## Key Metrics",
        "",
        f"- Direction recall: {summary['direction_recall']:.3f}",
        f"- IoU>=0.50 recall: {summary['iou50_recall']:.3f}",
        f"- Correct detections passing robot gates: {summary['robot_gate_correct_recall']:.3f}",
        f"- Correct detections passing laptop autonomous gates: {summary['laptop_actionable_correct_recall']:.3f}",
        f"- Autonomous FSM accepted recall: {summary['fsm_accepted_recall']:.3f}",
        f"- Robot-gate FSM accepted recall: {summary['robot_gate_fsm_accepted_recall']:.3f}",
        f"- False actionable non-signal images: {summary['false_actionable_on_non_signal']}",
        "",
        "## Thresholds",
        "",
    ]
    for key, value in summary["thresholds"].items():
        lines.append(f"- `{key}`: {value}")
    lines.extend(["", "## By Expected Class", ""])
    for key, value in summary["by_expected"].items():
        lines.append(f"- `{key}`: {value}")
    lines.extend(
        [
            "",
            "## FSM",
            "",
            f"- Autonomous states: {summary['fsm_states']}",
            f"- Autonomous YOLO status: {summary['fsm_yolo_status']}",
            f"- Autonomous rejections: {summary['fsm_rejection_reasons']}",
            f"- Robot-gate states: {summary['robot_gate_fsm_states']}",
            f"- Robot-gate YOLO status: {summary['robot_gate_fsm_yolo_status']}",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    dataset = (REPO_ROOT / args.dataset).resolve()
    config_path = (REPO_ROOT / args.config).resolve()
    model_path = (REPO_ROOT / args.model).resolve()
    paths = image_paths(dataset, args.split, args.limit)
    if not paths:
        print(f"no images found under {dataset}", file=sys.stderr)
        return 1
    if not model_path.exists():
        print(f"model not found: {model_path}", file=sys.stderr)
        return 1

    run_id = time.strftime("%Y%m%d_%H%M%S") + "_signal_fsm_dataset"
    out_dir = Path(args.out_dir) if args.out_dir else REPO_ROOT / "output" / "signal_fsm_eval" / run_id
    out_dir.mkdir(parents=True, exist_ok=True)

    from ultralytics import YOLO

    params = load_replay_profile("wall_follow", config_path=config_path)
    model = YOLO(str(model_path))
    rows: List[Dict[str, Any]] = []

    for image_path, result in zip(paths, model.predict(paths, imgsz=args.imgsz, conf=args.conf, verbose=False, stream=True)):
        labels = read_labels(label_path_for(dataset, image_path))
        expected = expected_direction(labels)
        detection = best_signal_detection(result)
        height, width = result.orig_shape[:2]
        best_iou = best_iou_for_detection(detection, labels, width, height)
        autonomous_fsm = fsm_response(detection, params, require_laptop_actionable=True)
        robot_gate_fsm = fsm_response(detection, params, require_laptop_actionable=False)
        rows.append(
            {
                "image": str(image_path.relative_to(REPO_ROOT)),
                "split": image_path.parent.name,
                "expected_direction": expected,
                "labels": labels,
                "detection": detection,
                "direction_correct": bool(detection and detection.get("direction") == expected and expected != "none"),
                "iou": round(best_iou, 4),
                "autonomous_fsm": autonomous_fsm,
                "robot_gate_fsm": robot_gate_fsm,
            }
        )

    summary = summarize(rows, params, args)
    write_jsonl(out_dir / "per_image.jsonl", rows)
    write_csv(out_dir / "per_image.csv", rows)
    write_json(out_dir / "summary.json", summary)
    write_markdown(out_dir / "summary.md", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"\nwrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
