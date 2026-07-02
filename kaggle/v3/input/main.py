import csv
import json
import shutil
import subprocess
import sys
import traceback
import zipfile
from collections import Counter
from pathlib import Path

subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "--upgrade", "ultralytics"])

import cv2
import numpy as np
from ultralytics import YOLO


SEED = 42
CLASSES = ["left-arrow", "right-arrow", "stop-signal"]
DATASET_REF = "jeffreyamc/yolo-signals-manual-gt"
DATASET_SLUG = "yolo-signals-manual-gt"
MODEL_NAME = "yolo26n.pt"
IMG_SIZE = 640

WORKING = Path("/kaggle/working")
RUNS = WORKING / "runs"
QUAL_DIR = WORKING / "qualitative_predictions"
SUMMARY_PATH = WORKING / "run_summary.json"
METRICS_PATH = WORKING / "metrics.json"
DATASET_SUMMARY_PATH = WORKING / "dataset_summary.json"
PRED_CSV = WORKING / "sample_predictions.csv"
GRID_PATH = WORKING / "qualitative_grid.jpg"
RUNTIME_DATA_YAML = WORKING / "data_v3_runtime.yaml"

progress = {
    "status": "running",
    "stage": "setup",
    "dataset": DATASET_REF,
    "model": MODEL_NAME,
    "classes": CLASSES,
    "artifacts": [],
}


def save_json(path, payload):
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")


def save_progress():
    save_json(SUMMARY_PATH, progress)


def record_artifact(path):
    path = Path(path)
    if path.exists():
        try:
            rel = str(path.relative_to(WORKING))
        except ValueError:
            rel = str(path)
        if rel not in progress["artifacts"]:
            progress["artifacts"].append(rel)


def find_data_root():
    candidates = [
        Path("/kaggle/input") / DATASET_SLUG,
        Path("/kaggle/input/datasets/jeffreyamc") / DATASET_SLUG,
    ]
    candidates.extend(p.parent for p in Path("/kaggle/input").glob("**/data.yaml"))
    seen = set()
    for root in candidates:
        root = root.resolve()
        if root in seen:
            continue
        seen.add(root)
        if not (root / "data.yaml").is_file():
            continue
        required = [
            root / "images" / "train",
            root / "images" / "val",
            root / "labels" / "train",
            root / "labels" / "val",
        ]
        if all(p.is_dir() for p in required):
            return root
    raise FileNotFoundError("Could not find manual YOLO dataset under /kaggle/input")


def image_files(split_dir):
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    return sorted(p for p in split_dir.rglob("*") if p.suffix.lower() in exts)


def label_path_for(data_root, image_path):
    split = image_path.parent.name
    return data_root / "labels" / split / f"{image_path.stem}.txt"


def read_yolo_label(label_path):
    rows = []
    if not label_path.is_file():
        return rows
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls, xc, yc, bw, bh = parts[:5]
        rows.append((int(float(cls)), float(xc), float(yc), float(bw), float(bh)))
    return rows


def yolo_to_xyxy(label, width, height):
    cls_id, xc, yc, bw, bh = label
    x1 = (xc - bw / 2.0) * width
    y1 = (yc - bh / 2.0) * height
    x2 = (xc + bw / 2.0) * width
    y2 = (yc + bh / 2.0) * height
    return cls_id, np.array([x1, y1, x2, y2], dtype=float)


def box_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = area_a + area_b - inter
    return float(inter / denom) if denom > 0 else 0.0


def draw_box(img, box, color, label):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    h, w = img.shape[:2]
    x1, x2 = max(0, min(w - 1, x1)), max(0, min(w - 1, x2))
    y1, y2 = max(0, min(h - 1, y1)), max(0, min(h - 1, y2))
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(img, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)


def summarize_dataset(data_root):
    summary = {"root": str(data_root), "splits": {}, "class_counts": {name: 0 for name in CLASSES}}
    for split in ["train", "val"]:
        imgs = image_files(data_root / "images" / split)
        missing_labels = 0
        label_count = 0
        split_classes = Counter()
        for img in imgs:
            labels = read_yolo_label(label_path_for(data_root, img))
            if not labels:
                missing_labels += 1
            label_count += len(labels)
            for label in labels:
                cls_id = label[0]
                if 0 <= cls_id < len(CLASSES):
                    split_classes[CLASSES[cls_id]] += 1
                    summary["class_counts"][CLASSES[cls_id]] += 1
        summary["splits"][split] = {
            "images": len(imgs),
            "labels": label_count,
            "missing_label_files_or_empty": missing_labels,
            "class_counts": dict(split_classes),
        }
    save_json(DATASET_SUMMARY_PATH, summary)
    record_artifact(DATASET_SUMMARY_PATH)
    return summary


def write_runtime_data_yaml(data_root):
    lines = [
        f"path: {data_root}",
        "train: images/train",
        "val: images/val",
        "names:",
    ]
    lines.extend(f"  {idx}: {name}" for idx, name in enumerate(CLASSES))
    RUNTIME_DATA_YAML.write_text("\n".join(lines) + "\n", encoding="utf-8")
    record_artifact(RUNTIME_DATA_YAML)
    return RUNTIME_DATA_YAML


def copy_if_exists(src, dst):
    src = Path(src)
    if src.exists():
        shutil.copy2(src, dst)
        record_artifact(dst)


def export_training_artifacts(save_dir):
    for name in [
        "args.yaml",
        "results.csv",
        "results.png",
        "confusion_matrix.png",
        "confusion_matrix_normalized.png",
        "labels.jpg",
        "labels_correlogram.jpg",
        "BoxF1_curve.png",
        "BoxP_curve.png",
        "BoxPR_curve.png",
        "BoxR_curve.png",
        "train_batch0.jpg",
        "train_batch1.jpg",
        "train_batch2.jpg",
        "val_batch0_labels.jpg",
        "val_batch0_pred.jpg",
        "val_batch1_labels.jpg",
        "val_batch1_pred.jpg",
        "val_batch2_labels.jpg",
        "val_batch2_pred.jpg",
    ]:
        copy_if_exists(save_dir / name, WORKING / name)


def make_qualitative_outputs(model, data_root, sample_count=12):
    QUAL_DIR.mkdir(parents=True, exist_ok=True)
    val_images = image_files(data_root / "images" / "val")
    if not val_images:
        raise RuntimeError("No validation images found for qualitative samples")

    # Deterministic spread through validation instead of only first contiguous frames.
    if len(val_images) <= sample_count:
        samples = val_images
    else:
        idxs = np.linspace(0, len(val_images) - 1, sample_count, dtype=int)
        samples = [val_images[int(i)] for i in idxs]

    csv_rows = []
    rendered = []
    for image_path in samples:
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        labels = read_yolo_label(label_path_for(data_root, image_path))
        gt = labels[0] if labels else None
        gt_cls = None
        gt_box = None
        gt_area_ratio = None
        if gt:
            gt_cls, gt_box = yolo_to_xyxy(gt, w, h)
            gt_area_ratio = float(((gt_box[2] - gt_box[0]) * (gt_box[3] - gt_box[1])) / max(1, w * h))

        pred = model.predict(str(image_path), imgsz=IMG_SIZE, conf=0.25, device=0, verbose=False)[0]
        best = None
        if pred.boxes is not None and len(pred.boxes) > 0:
            confs = pred.boxes.conf.detach().cpu().numpy()
            best_idx = int(np.argmax(confs))
            xyxy = pred.boxes.xyxy[best_idx].detach().cpu().numpy().astype(float)
            cls_id = int(pred.boxes.cls[best_idx].detach().cpu().item())
            conf = float(confs[best_idx])
            best = (cls_id, conf, xyxy)

        canvas = img.copy()
        if gt_box is not None:
            draw_box(canvas, gt_box, (255, 0, 0), f"GT {CLASSES[gt_cls]}")
        iou = None
        pred_cls_name = ""
        pred_conf = None
        if best:
            pred_cls, pred_conf, pred_box = best
            pred_cls_name = CLASSES[pred_cls] if 0 <= pred_cls < len(CLASSES) else str(pred_cls)
            draw_box(canvas, pred_box, (0, 180, 0), f"P {pred_cls_name} {pred_conf:.2f}")
            if gt_box is not None:
                iou = box_iou(gt_box, pred_box)

        out_name = f"{image_path.stem}_gt_pred.jpg"
        out_path = QUAL_DIR / out_name
        cv2.imwrite(str(out_path), canvas)
        rendered.append(canvas)
        csv_rows.append({
            "image": image_path.name,
            "gt_class": CLASSES[gt_cls] if gt_cls is not None else "",
            "gt_area_ratio": "" if gt_area_ratio is None else f"{gt_area_ratio:.6f}",
            "pred_class": pred_cls_name,
            "pred_conf": "" if pred_conf is None else f"{pred_conf:.6f}",
            "pred_iou_vs_gt": "" if iou is None else f"{iou:.6f}",
            "qualitative_image": out_name,
        })

    with PRED_CSV.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()) if csv_rows else [
            "image", "gt_class", "gt_area_ratio", "pred_class", "pred_conf", "pred_iou_vs_gt", "qualitative_image"
        ])
        writer.writeheader()
        writer.writerows(csv_rows)
    record_artifact(PRED_CSV)

    if rendered:
        thumbs = []
        for img in rendered:
            thumb = cv2.resize(img, (320, 320), interpolation=cv2.INTER_AREA)
            thumbs.append(thumb)
        cols = 4
        rows = int(np.ceil(len(thumbs) / cols))
        blank = np.full_like(thumbs[0], 255)
        grid_rows = []
        for r in range(rows):
            row_imgs = thumbs[r * cols:(r + 1) * cols]
            row_imgs.extend([blank.copy()] * (cols - len(row_imgs)))
            grid_rows.append(np.hstack(row_imgs))
        grid = np.vstack(grid_rows)
        cv2.imwrite(str(GRID_PATH), grid)
        record_artifact(GRID_PATH)

    zip_path = WORKING / "qualitative_predictions.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(QUAL_DIR.glob("*.jpg")):
            zf.write(path, arcname=path.name)
    record_artifact(zip_path)

    return csv_rows


def metrics_payload(metrics):
    payload = {
        "map50": float(metrics.box.map50),
        "map50_95": float(metrics.box.map),
        "precision": float(metrics.box.mp),
        "recall": float(metrics.box.mr),
    }
    try:
        payload["fitness"] = float(metrics.fitness)
    except Exception:
        pass
    return payload


def main():
    WORKING.mkdir(parents=True, exist_ok=True)
    progress["stage"] = "locate_dataset"
    save_progress()

    data_root = find_data_root()
    data_yaml = write_runtime_data_yaml(data_root)
    progress["data_root"] = str(data_root)
    progress["data_yaml"] = str(data_yaml)
    dataset_summary = summarize_dataset(data_root)
    progress["dataset_summary"] = dataset_summary
    save_progress()

    progress["stage"] = "train"
    save_progress()
    model = YOLO(MODEL_NAME)
    train_results = model.train(
        data=str(data_yaml),
        epochs=80,
        imgsz=IMG_SIZE,
        batch=16,
        patience=12,
        device=0,
        seed=SEED,
        workers=2,
        cache=True,
        project=str(RUNS),
        name="yolo26n_det_v3_manual",
        exist_ok=True,
        verbose=True,
    )

    save_dir = Path(getattr(train_results, "save_dir", RUNS / "yolo26n_det_v3_manual"))
    weights_dir = save_dir / "weights"
    best_src = weights_dir / "best.pt"
    last_src = weights_dir / "last.pt"
    best_out = WORKING / "best.pt"
    last_out = WORKING / "last.pt"
    copy_if_exists(best_src, best_out)
    copy_if_exists(last_src, last_out)
    export_training_artifacts(save_dir)

    progress["stage"] = "validate"
    save_progress()
    trained = YOLO(str(best_out if best_out.exists() else best_src))
    val_metrics = trained.val(data=str(data_yaml), imgsz=IMG_SIZE, device=0, verbose=False)
    metric_data = metrics_payload(val_metrics)
    metric_data["model"] = MODEL_NAME
    metric_data["imgsz"] = IMG_SIZE
    metric_data["epochs_requested"] = 80
    metric_data["classes"] = CLASSES
    save_json(METRICS_PATH, metric_data)
    record_artifact(METRICS_PATH)

    progress["stage"] = "qualitative_samples"
    save_progress()
    qualitative_rows = make_qualitative_outputs(trained, data_root, sample_count=12)

    for path in [SUMMARY_PATH, DATASET_SUMMARY_PATH, METRICS_PATH, PRED_CSV, GRID_PATH]:
        record_artifact(path)
    progress["stage"] = "complete"
    progress["status"] = "complete"
    progress["metrics"] = metric_data
    progress["qualitative_sample_count"] = len(qualitative_rows)
    save_progress()

    pretrained = WORKING / MODEL_NAME
    if pretrained.exists():
        pretrained.unlink()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        pretrained = WORKING / MODEL_NAME
        if pretrained.exists():
            pretrained.unlink()
        progress["status"] = "failed"
        progress["stage"] = "error"
        progress["error"] = repr(exc)
        progress["traceback"] = traceback.format_exc()
        save_progress()
        print(progress["traceback"])
        sys.exit(1)
