import csv
import json
import math
import random
import shutil
import subprocess
import sys
import traceback
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "--upgrade", "ultralytics"])

import cv2
import numpy as np
import yaml
from ultralytics import YOLO


SEED = 42
random.seed(SEED)
np.random.seed(SEED)

DATASET_REF = "jeffreyamc/yolo-signals-manual-gt"
DATASET_SLUG = "yolo-signals-manual-gt"
MODEL_NAME = "yolo26n.pt"
IMG_SIZE = 640
EPOCHS = 120
TRAIN_NAME = "yolo26n_det_v5_stop_rot_composite"
STOP_CLASS_NAME = "stop-signal"
STOP_ROTATION_ANGLES = [-24, -18, -12, 12, 18, 24]
COMPOSITE_TRAIN_IMAGES = 36
COMPOSITE_EVAL_IMAGES = 12

# Keep the v4 non-rotation augmentation style. Stop rotations are materialized
# offline below so arrows are not globally rotated by the dataloader.
AUGMENT_ARGS = {
    "hsv_h": 0.015,
    "hsv_s": 0.50,
    "hsv_v": 0.35,
    "degrees": 0.0,
    "translate": 0.08,
    "scale": 0.35,
    "shear": 0.0,
    "fliplr": 0.0,
    "flipud": 0.0,
    "mosaic": 0.70,
    "mixup": 0.05,
    "copy_paste": 0.0,
    "close_mosaic": 10,
}

WORKING = Path("/kaggle/working")
TEMP = Path("/kaggle/temp")
PREPARED = TEMP / "yolo_signals_v5_prepared"
RUNS = WORKING / "runs"
QUAL_DIR = WORKING / "qualitative_predictions"
COMPOSITE_EVAL_DIR = WORKING / "composite_eval"
COMPOSITE_PRED_DIR = WORKING / "composite_predictions"
SUMMARY_PATH = WORKING / "run_summary.json"
METRICS_PATH = WORKING / "metrics.json"
DATASET_SUMMARY_PATH = WORKING / "dataset_summary.json"
AUGMENT_SUMMARY_PATH = WORKING / "augmentation_summary.json"
PRED_CSV = WORKING / "sample_predictions.csv"
COMPOSITE_CSV = WORKING / "composite_eval_predictions.csv"
GRID_PATH = WORKING / "qualitative_grid.jpg"
COMPOSITE_GRID_PATH = WORKING / "composite_eval_grid.jpg"
RUNTIME_DATA_YAML = WORKING / "data_v5_runtime.yaml"

progress = {
    "status": "running",
    "stage": "setup",
    "dataset": DATASET_REF,
    "model": MODEL_NAME,
    "stop_rotation_angles": STOP_ROTATION_ANGLES,
    "augmentation": AUGMENT_ARGS,
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
    raise FileNotFoundError("Could not find YOLO dataset under /kaggle/input")


def load_class_names(data_yaml):
    data = yaml.safe_load(Path(data_yaml).read_text(encoding="utf-8"))
    names = data["names"]
    if isinstance(names, dict):
        return [str(names[idx]) for idx in sorted(names)]
    return [str(name) for name in names]


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


def write_yolo_labels(label_path, labels):
    lines = [f"{cls} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}" for cls, xc, yc, bw, bh in labels]
    label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def yolo_to_xyxy(label, width, height):
    cls_id, xc, yc, bw, bh = label
    x1 = (xc - bw / 2.0) * width
    y1 = (yc - bh / 2.0) * height
    x2 = (xc + bw / 2.0) * width
    y2 = (yc + bh / 2.0) * height
    return cls_id, np.array([x1, y1, x2, y2], dtype=float)


def xyxy_to_yolo(cls_id, box, width, height):
    x1, y1, x2, y2 = box
    x1, x2 = np.clip([x1, x2], 0, width - 1)
    y1, y2 = np.clip([y1, y2], 0, height - 1)
    if x2 <= x1 or y2 <= y1:
        return None
    xc = ((x1 + x2) / 2.0) / width
    yc = ((y1 + y2) / 2.0) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return int(cls_id), float(xc), float(yc), float(bw), float(bh)


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
    cv2.putText(img, label[:32], (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def copy_dataset_to_prepared(source_root):
    if PREPARED.exists():
        shutil.rmtree(PREPARED)
    for rel in ["images/train", "images/val", "labels/train", "labels/val"]:
        shutil.copytree(source_root / rel, PREPARED / rel)


def rotate_labels(labels, width, height, matrix):
    transformed = []
    for label in labels:
        cls_id, box = yolo_to_xyxy(label, width, height)
        x1, y1, x2, y2 = box
        points = np.array([[x1, y1, 1], [x2, y1, 1], [x2, y2, 1], [x1, y2, 1]], dtype=float)
        rotated = (matrix @ points.T).T
        rx1, ry1 = rotated[:, 0].min(), rotated[:, 1].min()
        rx2, ry2 = rotated[:, 0].max(), rotated[:, 1].max()
        converted = xyxy_to_yolo(cls_id, [rx1, ry1, rx2, ry2], width, height)
        if converted is not None:
            transformed.append(converted)
    return transformed


def add_stop_rotations(classes):
    stop_id = classes.index(STOP_CLASS_NAME)
    stats = {
        "angles": STOP_ROTATION_ANGLES,
        "source_images": 0,
        "created_images": 0,
        "skipped_mixed_labels": 0,
        "skipped_unreadable": 0,
    }
    for label_path in sorted((PREPARED / "labels" / "train").glob("*.txt")):
        labels = read_yolo_label(label_path)
        if not labels or not any(label[0] == stop_id for label in labels):
            continue
        if any(label[0] != stop_id for label in labels):
            stats["skipped_mixed_labels"] += 1
            continue
        image_path = PREPARED / "images" / "train" / f"{label_path.stem}.jpg"
        if not image_path.exists():
            matches = list((PREPARED / "images" / "train").glob(f"{label_path.stem}.*"))
            image_path = matches[0] if matches else image_path
        img = cv2.imread(str(image_path))
        if img is None:
            stats["skipped_unreadable"] += 1
            continue
        stats["source_images"] += 1
        h, w = img.shape[:2]
        center = (w / 2.0, h / 2.0)
        for angle in STOP_ROTATION_ANGLES:
            matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            rotated = cv2.warpAffine(img, matrix, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
            rotated_labels = rotate_labels(labels, w, h, matrix)
            if not rotated_labels:
                continue
            suffix = f"stoprot_{angle:+d}".replace("+", "p").replace("-", "m")
            out_stem = f"{label_path.stem}_{suffix}"
            out_img = PREPARED / "images" / "train" / f"{out_stem}.jpg"
            out_lbl = PREPARED / "labels" / "train" / f"{out_stem}.txt"
            cv2.imwrite(str(out_img), rotated)
            write_yolo_labels(out_lbl, rotated_labels)
            stats["created_images"] += 1
    return stats


def collect_single_object_sources(data_root, split, classes):
    by_class = defaultdict(list)
    for image_path in image_files(data_root / "images" / split):
        labels = read_yolo_label(label_path_for(data_root, image_path))
        if len(labels) != 1:
            continue
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        cls_id, box = yolo_to_xyxy(labels[0], img.shape[1], img.shape[0])
        by_class[cls_id].append((image_path, labels[0]))
    return by_class


def crop_from_source(image_path, label):
    img = cv2.imread(str(image_path))
    if img is None:
        return None
    h, w = img.shape[:2]
    cls_id, box = yolo_to_xyxy(label, w, h)
    x1, y1, x2, y2 = box
    pad = 0.18 * max(x2 - x1, y2 - y1)
    x1 = int(max(0, math.floor(x1 - pad)))
    y1 = int(max(0, math.floor(y1 - pad)))
    x2 = int(min(w - 1, math.ceil(x2 + pad)))
    y2 = int(min(h - 1, math.ceil(y2 + pad)))
    if x2 <= x1 or y2 <= y1:
        return None
    crop = img[y1:y2, x1:x2].copy()
    local_box = np.array([0, 0, crop.shape[1] - 1, crop.shape[0] - 1], dtype=float)
    return int(cls_id), crop, local_box


def make_composite_image(sources_by_class, classes, out_img, out_lbl, wanted_class_ids=None):
    class_ids = wanted_class_ids or [idx for idx in range(len(classes)) if sources_by_class.get(idx)]
    if len(class_ids) < 2:
        return False
    chosen_ids = random.sample(class_ids, k=min(random.choice([2, 3]), len(class_ids)))
    canvas = np.full((IMG_SIZE, IMG_SIZE, 3), 235, dtype=np.uint8)
    slots = [(32, 32, 288, 288), (352, 32, 608, 288), (192, 352, 448, 608)]
    labels = []
    for cls_id, slot in zip(chosen_ids, slots):
        source = random.choice(sources_by_class[cls_id])
        item = crop_from_source(*source)
        if item is None:
            continue
        cls_id, crop, _ = item
        sx1, sy1, sx2, sy2 = slot
        max_w, max_h = sx2 - sx1, sy2 - sy1
        scale = min(max_w / crop.shape[1], max_h / crop.shape[0]) * random.uniform(0.72, 0.96)
        new_w = max(24, int(crop.shape[1] * scale))
        new_h = max(24, int(crop.shape[0] * scale))
        resized = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)
        ox = sx1 + random.randint(0, max(0, max_w - new_w))
        oy = sy1 + random.randint(0, max(0, max_h - new_h))
        canvas[oy:oy + new_h, ox:ox + new_w] = resized
        label = xyxy_to_yolo(cls_id, [ox, oy, ox + new_w, oy + new_h], IMG_SIZE, IMG_SIZE)
        if label is not None:
            labels.append(label)
    if len(labels) < 2:
        return False
    cv2.imwrite(str(out_img), canvas)
    write_yolo_labels(out_lbl, labels)
    return True


def add_training_composites(classes):
    sources = collect_single_object_sources(PREPARED, "train", classes)
    preferred = [classes.index(name) for name in ["left-arrow", "right-arrow", STOP_CLASS_NAME] if name in classes]
    if len(preferred) < 2:
        preferred = [idx for idx in range(len(classes)) if sources.get(idx)]
    stats = {"requested": COMPOSITE_TRAIN_IMAGES, "created": 0, "classes_used": [classes[idx] for idx in preferred]}
    for idx in range(COMPOSITE_TRAIN_IMAGES * 2):
        if stats["created"] >= COMPOSITE_TRAIN_IMAGES:
            break
        stem = f"composite_train_{stats['created']:04d}"
        if make_composite_image(
            sources,
            classes,
            PREPARED / "images" / "train" / f"{stem}.jpg",
            PREPARED / "labels" / "train" / f"{stem}.txt",
            preferred,
        ):
            stats["created"] += 1
    return stats


def make_composite_eval_set(data_root, classes):
    COMPOSITE_EVAL_DIR.mkdir(parents=True, exist_ok=True)
    sources = collect_single_object_sources(data_root, "val", classes)
    preferred = [classes.index(name) for name in ["left-arrow", "right-arrow", STOP_CLASS_NAME] if name in classes]
    if len(preferred) < 2:
        preferred = [idx for idx in range(len(classes)) if sources.get(idx)]
    created = 0
    for idx in range(COMPOSITE_EVAL_IMAGES * 2):
        if created >= COMPOSITE_EVAL_IMAGES:
            break
        stem = f"composite_eval_{created:04d}"
        if make_composite_image(
            sources,
            classes,
            COMPOSITE_EVAL_DIR / f"{stem}.jpg",
            COMPOSITE_EVAL_DIR / f"{stem}.txt",
            preferred,
        ):
            created += 1
    return {"requested": COMPOSITE_EVAL_IMAGES, "created": created, "classes_used": [classes[idx] for idx in preferred]}


def summarize_dataset(data_root, classes):
    summary = {"root": str(data_root), "splits": {}, "class_counts": {name: 0 for name in classes}}
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
                if 0 <= cls_id < len(classes):
                    split_classes[classes[cls_id]] += 1
                    summary["class_counts"][classes[cls_id]] += 1
        summary["splits"][split] = {
            "images": len(imgs),
            "labels": label_count,
            "missing_label_files_or_empty": missing_labels,
            "class_counts": dict(split_classes),
        }
    save_json(DATASET_SUMMARY_PATH, summary)
    record_artifact(DATASET_SUMMARY_PATH)
    return summary


def write_runtime_data_yaml(data_root, classes):
    lines = [f"path: {data_root}", "train: images/train", "val: images/val", "names:"]
    lines.extend(f"  {idx}: {name}" for idx, name in enumerate(classes))
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
        "args.yaml", "results.csv", "results.png", "confusion_matrix.png",
        "confusion_matrix_normalized.png", "labels.jpg", "labels_correlogram.jpg",
        "BoxF1_curve.png", "BoxP_curve.png", "BoxPR_curve.png", "BoxR_curve.png",
        "train_batch0.jpg", "train_batch1.jpg", "train_batch2.jpg",
        "val_batch0_labels.jpg", "val_batch0_pred.jpg", "val_batch1_labels.jpg",
        "val_batch1_pred.jpg", "val_batch2_labels.jpg", "val_batch2_pred.jpg",
    ]:
        copy_if_exists(save_dir / name, WORKING / name)


def make_qualitative_outputs(model, data_root, classes, sample_count=16):
    QUAL_DIR.mkdir(parents=True, exist_ok=True)
    val_images = image_files(data_root / "images" / "val")
    if not val_images:
        raise RuntimeError("No validation images found")
    by_class = {name: [] for name in classes}
    for image_path in val_images:
        labels = read_yolo_label(label_path_for(data_root, image_path))
        if labels and 0 <= labels[0][0] < len(classes):
            by_class[classes[labels[0][0]]].append(image_path)
    samples = []
    per_class = max(1, sample_count // len(classes))
    for class_name in classes:
        class_images = by_class[class_name]
        if not class_images:
            continue
        idxs = np.linspace(0, len(class_images) - 1, min(per_class, len(class_images)), dtype=int)
        samples.extend(class_images[int(i)] for i in idxs)
    remaining = [path for path in val_images if path not in set(samples)]
    if len(samples) < sample_count and remaining:
        idxs = np.linspace(0, len(remaining) - 1, min(sample_count - len(samples), len(remaining)), dtype=int)
        samples.extend(remaining[int(i)] for i in idxs)
    samples = samples[:sample_count]

    csv_rows, rendered = [], []
    for image_path in samples:
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        labels = read_yolo_label(label_path_for(data_root, image_path))
        gt = labels[0] if labels else None
        gt_cls, gt_box, gt_area_ratio = None, None, None
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
            draw_box(canvas, gt_box, (255, 0, 0), f"GT {classes[gt_cls]}")
        iou, pred_cls_name, pred_conf = None, "", None
        if best:
            pred_cls, pred_conf, pred_box = best
            pred_cls_name = classes[pred_cls] if 0 <= pred_cls < len(classes) else str(pred_cls)
            draw_box(canvas, pred_box, (0, 180, 0), f"P {pred_cls_name} {pred_conf:.2f}")
            if gt_box is not None:
                iou = box_iou(gt_box, pred_box)
        out_name = f"{image_path.stem}_gt_pred.jpg"
        cv2.imwrite(str(QUAL_DIR / out_name), canvas)
        rendered.append(canvas)
        csv_rows.append({
            "image": image_path.name,
            "gt_class": classes[gt_cls] if gt_cls is not None else "",
            "gt_area_ratio": "" if gt_area_ratio is None else f"{gt_area_ratio:.6f}",
            "pred_class": pred_cls_name,
            "pred_conf": "" if pred_conf is None else f"{pred_conf:.6f}",
            "pred_iou_vs_gt": "" if iou is None else f"{iou:.6f}",
            "qualitative_image": out_name,
        })
    write_csv(PRED_CSV, csv_rows)
    record_artifact(PRED_CSV)
    write_grid(rendered, GRID_PATH)
    return csv_rows


def evaluate_composites(model, classes):
    COMPOSITE_PRED_DIR.mkdir(parents=True, exist_ok=True)
    rows, rendered = [], []
    for image_path in sorted(COMPOSITE_EVAL_DIR.glob("*.jpg")):
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        labels = read_yolo_label(COMPOSITE_EVAL_DIR / f"{image_path.stem}.txt")
        canvas = img.copy()
        gt_classes = []
        for label in labels:
            cls_id, box = yolo_to_xyxy(label, w, h)
            gt_classes.append(classes[cls_id])
            draw_box(canvas, box, (255, 0, 0), f"GT {classes[cls_id]}")
        pred = model.predict(str(image_path), imgsz=IMG_SIZE, conf=0.25, device=0, verbose=False)[0]
        pred_classes = []
        if pred.boxes is not None:
            for box, conf, cls_id in zip(
                pred.boxes.xyxy.detach().cpu().numpy(),
                pred.boxes.conf.detach().cpu().numpy(),
                pred.boxes.cls.detach().cpu().numpy().astype(int),
            ):
                name = classes[cls_id] if 0 <= cls_id < len(classes) else str(cls_id)
                pred_classes.append(name)
                draw_box(canvas, box, (0, 180, 0), f"P {name} {float(conf):.2f}")
        out_name = f"{image_path.stem}_pred.jpg"
        cv2.imwrite(str(COMPOSITE_PRED_DIR / out_name), canvas)
        rendered.append(canvas)
        rows.append({
            "image": image_path.name,
            "gt_classes": "|".join(gt_classes),
            "pred_classes": "|".join(pred_classes),
            "gt_count": len(gt_classes),
            "pred_count": len(pred_classes),
            "all_gt_classes_detected": all(cls in pred_classes for cls in gt_classes),
            "qualitative_image": out_name,
        })
    write_csv(COMPOSITE_CSV, rows)
    record_artifact(COMPOSITE_CSV)
    write_grid(rendered, COMPOSITE_GRID_PATH)
    return rows


def write_csv(path, rows):
    fieldnames = list(rows[0].keys()) if rows else ["image"]
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_grid(images, path):
    if not images:
        return
    thumbs = [cv2.resize(img, (320, 320), interpolation=cv2.INTER_AREA) for img in images]
    cols = 4
    rows = int(np.ceil(len(thumbs) / cols))
    blank = np.full_like(thumbs[0], 255)
    grid_rows = []
    for r in range(rows):
        row_imgs = thumbs[r * cols:(r + 1) * cols]
        row_imgs.extend([blank.copy()] * (cols - len(row_imgs)))
        grid_rows.append(np.hstack(row_imgs))
    cv2.imwrite(str(path), np.vstack(grid_rows))
    record_artifact(path)


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


def make_zip(zip_path, paths):
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in paths:
            path = Path(path)
            if path.exists():
                if path.is_dir():
                    for child in sorted(path.rglob("*")):
                        if child.is_file():
                            zf.write(child, arcname=str(child.relative_to(path.parent)))
                else:
                    zf.write(path, arcname=path.name)
    record_artifact(zip_path)


def main():
    WORKING.mkdir(parents=True, exist_ok=True)
    progress["stage"] = "locate_dataset"
    save_progress()

    source_root = find_data_root()
    classes = load_class_names(source_root / "data.yaml")
    if STOP_CLASS_NAME not in classes:
        raise RuntimeError(f"Dataset classes {classes} do not include {STOP_CLASS_NAME}")
    progress["classes"] = classes
    progress["source_root"] = str(source_root)
    save_progress()

    progress["stage"] = "prepare_augmented_dataset"
    save_progress()
    copy_dataset_to_prepared(source_root)
    stop_rotation_stats = add_stop_rotations(classes)
    composite_train_stats = add_training_composites(classes)
    composite_eval_stats = make_composite_eval_set(PREPARED, classes)
    augmentation_summary = {
        "stop_rotations": stop_rotation_stats,
        "training_composites": composite_train_stats,
        "composite_eval": composite_eval_stats,
        "note": "Ultralytics degrees is 0.0; stop-signal rotation augmentation is materialized offline only for pure stop-signal train images.",
    }
    save_json(AUGMENT_SUMMARY_PATH, augmentation_summary)
    record_artifact(AUGMENT_SUMMARY_PATH)
    progress["augmentation_summary"] = augmentation_summary
    save_progress()

    data_yaml = write_runtime_data_yaml(PREPARED, classes)
    dataset_summary = summarize_dataset(PREPARED, classes)
    progress["data_root"] = str(PREPARED)
    progress["data_yaml"] = str(data_yaml)
    progress["dataset_summary"] = dataset_summary
    save_progress()

    progress["stage"] = "train"
    save_progress()
    model = YOLO(MODEL_NAME)
    train_results = model.train(
        data=str(data_yaml),
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        batch=16,
        patience=20,
        device=0,
        seed=SEED,
        workers=2,
        cache=True,
        project=str(RUNS),
        name=TRAIN_NAME,
        exist_ok=True,
        verbose=True,
        **AUGMENT_ARGS,
    )

    save_dir = Path(getattr(train_results, "save_dir", RUNS / TRAIN_NAME))
    best_src = save_dir / "weights" / "best.pt"
    last_src = save_dir / "weights" / "last.pt"
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
    metric_data["epochs_requested"] = EPOCHS
    metric_data["classes"] = classes
    metric_data["augmentation"] = AUGMENT_ARGS
    metric_data["stop_rotation_angles"] = STOP_ROTATION_ANGLES
    try:
        maps = list(getattr(val_metrics.box, "maps", []))
        metric_data["map50_95_per_class"] = {classes[idx]: float(value) for idx, value in enumerate(maps[:len(classes)])}
    except Exception:
        pass
    save_json(METRICS_PATH, metric_data)
    record_artifact(METRICS_PATH)

    progress["stage"] = "qualitative_samples"
    save_progress()
    qualitative_rows = make_qualitative_outputs(trained, PREPARED, classes, sample_count=16)
    composite_rows = evaluate_composites(trained, classes)

    make_zip(WORKING / "qualitative_predictions.zip", [QUAL_DIR])
    make_zip(WORKING / "composite_eval_predictions.zip", [COMPOSITE_PRED_DIR, COMPOSITE_EVAL_DIR, COMPOSITE_CSV, COMPOSITE_GRID_PATH])
    make_zip(WORKING / "qualitative_results.zip", [QUAL_DIR, PRED_CSV, GRID_PATH, COMPOSITE_PRED_DIR, COMPOSITE_CSV, COMPOSITE_GRID_PATH])
    make_zip(WORKING / "quantitative_metrics.zip", [
        METRICS_PATH, DATASET_SUMMARY_PATH, AUGMENT_SUMMARY_PATH, save_dir / "results.csv",
        WORKING / "results.csv", WORKING / "results.png", WORKING / "confusion_matrix.png",
        WORKING / "confusion_matrix_normalized.png", WORKING / "BoxF1_curve.png",
        WORKING / "BoxP_curve.png", WORKING / "BoxPR_curve.png", WORKING / "BoxR_curve.png",
    ])

    for path in [SUMMARY_PATH, DATASET_SUMMARY_PATH, AUGMENT_SUMMARY_PATH, METRICS_PATH, PRED_CSV, COMPOSITE_CSV, GRID_PATH, COMPOSITE_GRID_PATH]:
        record_artifact(path)
    progress["stage"] = "complete"
    progress["status"] = "complete"
    progress["metrics"] = metric_data
    progress["qualitative_sample_count"] = len(qualitative_rows)
    progress["composite_eval_sample_count"] = len(composite_rows)
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
