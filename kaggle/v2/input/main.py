import csv
import json
import random
import shutil
import subprocess
import sys
import traceback
import zipfile
from pathlib import Path

subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "--upgrade", "ultralytics"])

import cv2
import numpy as np
from ultralytics import YOLO

SEED = 42
CLASSES = ["left-arrow", "right-arrow", "stop-signal"]
CLASS_TO_ID = {name: idx for idx, name in enumerate(CLASSES)}
DATASET_SLUG = "yolo-signals-input"
MODEL_NAME = "yolo26n.pt"
IMG_SIZE = 640

WORKING = Path("/kaggle/working")
TEMP = Path("/kaggle/temp")
PREPARED = TEMP / "yolo_signals_det"
RUNS = WORKING / "runs"
QUAL_DIR = WORKING / "qualitative_predictions"
SUMMARY_PATH = WORKING / "run_summary.json"
METRICS_PATH = WORKING / "metrics.json"
PSEUDO_CSV = WORKING / "pseudo_labels.csv"
PRED_CSV = WORKING / "sample_predictions.csv"
STATS_PATH = WORKING / "pseudo_label_stats.json"

progress = {
    "status": "running",
    "stage": "setup",
    "dataset": "jeffreyamc/yolo-signals-input",
    "model": MODEL_NAME,
    "classes": CLASSES,
    "artifacts": [],
}


def save_progress():
    SUMMARY_PATH.write_text(json.dumps(progress, indent=2), encoding="utf-8")


def record_artifact(path):
    path = Path(path)
    if path.exists():
        rel = str(path.relative_to(WORKING)) if path.is_relative_to(WORKING) else str(path)
        if rel not in progress["artifacts"]:
            progress["artifacts"].append(rel)


def find_data_root():
    candidates = [
        Path("/kaggle/input") / DATASET_SLUG,
        Path("/kaggle/input/datasets/jeffreyamc") / DATASET_SLUG,
    ]
    candidates.extend(p.parent for p in Path("/kaggle/input").glob("**/left-arrow") if p.is_dir())
    for root in candidates:
        if all((root / name).is_dir() for name in CLASSES):
            return root
    raise FileNotFoundError("Could not find class folders under /kaggle/input")


def pseudo_box(image_path):
    img = cv2.imread(str(image_path))
    if img is None:
        return None

    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = (gray < 95).astype("uint8") * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    for label_id in range(1, n_labels):
        x, y, bw, bh, area = stats[label_id]
        touches_bottom = y + bh >= h - 2
        too_wide = bw > 0.98 * w
        too_small = area < 25
        horizontal_band = bh < 0.10 * h and bw > 0.20 * w
        if touches_bottom or too_wide or too_small or horizontal_band:
            continue
        candidates.append((area, x, y, bw, bh))

    if not candidates:
        return None

    _area, x, y, bw, bh = max(candidates, key=lambda item: item[0])
    pad = round(max(bw, bh) * 0.10)
    x1 = max(0, int(x - pad))
    y1 = max(0, int(y - pad))
    x2 = min(w - 1, int(x + bw - 1 + pad))
    y2 = min(h - 1, int(y + bh - 1 + pad))
    return x1, y1, x2, y2, w, h


def median_fallback_box(norm_box, image_path):
    img = cv2.imread(str(image_path))
    if img is None:
        return None
    h, w = img.shape[:2]
    x1, y1, x2, y2 = norm_box
    return (
        max(0, int(round(x1 * w))),
        max(0, int(round(y1 * h))),
        min(w - 1, int(round(x2 * w))),
        min(h - 1, int(round(y2 * h))),
        w,
        h,
    )


def xyxy_to_yolo(box):
    x1, y1, x2, y2, w, h = box
    cx = ((x1 + x2) / 2) / w
    cy = ((y1 + y2) / 2) / h
    bw = (x2 - x1 + 1) / w
    bh = (y2 - y1 + 1) / h
    return cx, cy, bw, bh


def box_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1 + 1), max(0, iy2 - iy1 + 1)
    inter = iw * ih
    area_a = max(0, ax2 - ax1 + 1) * max(0, ay2 - ay1 + 1)
    area_b = max(0, bx2 - bx1 + 1) * max(0, by2 - by1 + 1)
    union = area_a + area_b - inter
    return inter / union if union else 0.0


def write_csv(path, rows, fields):
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def draw_box(img, box, label, color):
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(img, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


def make_grid(image_paths, out_path, cell=250, cols=4):
    imgs = []
    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            continue
        imgs.append(cv2.resize(img, (cell, cell)))
    if not imgs:
        return
    rows = []
    for i in range(0, len(imgs), cols):
        chunk = imgs[i : i + cols]
        while len(chunk) < cols:
            chunk.append(np.full((cell, cell, 3), 255, dtype=np.uint8))
        rows.append(np.hstack(chunk))
    cv2.imwrite(str(out_path), np.vstack(rows))


def zip_dir(src_dir, zip_path):
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(Path(src_dir).glob("*.jpg")):
            zf.write(path, path.name)


save_progress()

try:
    progress["stage"] = "resolve_dataset"
    save_progress()
    data_root = find_data_root()
    source_counts = {name: len(list((data_root / name).glob("*.jpg"))) for name in CLASSES}
    progress["data_root"] = str(data_root)
    progress["source_counts"] = source_counts
    save_progress()

    progress["stage"] = "pseudo_label"
    save_progress()
    if PREPARED.exists():
        shutil.rmtree(PREPARED)
    records = []
    skipped = []
    random.seed(SEED)

    detected_by_class = {class_name: [] for class_name in CLASSES}
    detected_cache = {}
    for class_name in CLASSES:
        for image_path in sorted((data_root / class_name).glob("*.jpg")):
            box = pseudo_box(image_path)
            if box is None:
                continue
            detected_cache[str(image_path)] = box
            x1, y1, x2, y2, w, h = box
            detected_by_class[class_name].append([x1 / w, y1 / h, x2 / w, y2 / h])

    fallback_by_class = {}
    for class_name, boxes in detected_by_class.items():
        if not boxes:
            raise RuntimeError(f"No pseudo-label boxes found for class {class_name}")
        fallback_by_class[class_name] = np.median(np.array(boxes), axis=0).tolist()

    for class_name in CLASSES:
        images = sorted((data_root / class_name).glob("*.jpg"))
        random.shuffle(images)
        val_count = max(1, round(len(images) * 0.2))
        val_names = {p.name for p in images[:val_count]}
        for image_path in images:
            split = "val" if image_path.name in val_names else "train"
            box = detected_cache.get(str(image_path))
            label_method = "detected"
            if box is None:
                box = median_fallback_box(fallback_by_class[class_name], image_path)
                label_method = "fallback_median"
            if box is None:
                skipped.append({"image": str(image_path), "class": class_name})
                continue

            x1, y1, x2, y2, w, h = box
            cx, cy, bw, bh = xyxy_to_yolo(box)
            out_name = f"{class_name}_{image_path.name}"
            img_out = PREPARED / "images" / split / out_name
            label_out = PREPARED / "labels" / split / out_name.replace(".jpg", ".txt")
            img_out.parent.mkdir(parents=True, exist_ok=True)
            label_out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(image_path, img_out)
            label_out.write_text(f"{CLASS_TO_ID[class_name]} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n", encoding="utf-8")
            records.append({
                "split": split,
                "class": class_name,
                "class_id": CLASS_TO_ID[class_name],
                "source": str(image_path),
                "image": str(img_out),
                "label": str(label_out),
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "width": w,
                "height": h,
                "area_ratio": ((x2 - x1 + 1) * (y2 - y1 + 1)) / (w * h),
                "label_method": label_method,
            })

    if len(records) != sum(source_counts.values()):
        raise RuntimeError(f"Pseudo-label failures: kept={len(records)} skipped={len(skipped)}")

    data_yaml = PREPARED / "data.yaml"
    data_yaml.write_text(
        "path: " + str(PREPARED) + "\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        + "".join(f"  {idx}: {name}\n" for idx, name in enumerate(CLASSES)),
        encoding="utf-8",
    )

    fields = ["split", "class", "class_id", "source", "image", "label", "x1", "y1", "x2", "y2", "width", "height", "area_ratio", "label_method"]
    write_csv(PSEUDO_CSV, records, fields)
    record_artifact(PSEUDO_CSV)

    stats = {
        "kept": len(records),
        "skipped": skipped,
        "source_counts": source_counts,
        "fallback_norm_xyxy": fallback_by_class,
        "by_class": {},
    }
    for class_name in CLASSES:
        class_rows = [r for r in records if r["class"] == class_name]
        areas = [r["area_ratio"] for r in class_rows]
        stats["by_class"][class_name] = {
            "count": len(class_rows),
            "train": sum(1 for r in class_rows if r["split"] == "train"),
            "val": sum(1 for r in class_rows if r["split"] == "val"),
            "detected": sum(1 for r in class_rows if r["label_method"] == "detected"),
            "fallback_median": sum(1 for r in class_rows if r["label_method"] == "fallback_median"),
            "area_ratio_min": min(areas),
            "area_ratio_mean": sum(areas) / len(areas),
            "area_ratio_max": max(areas),
        }
    STATS_PATH.write_text(json.dumps(stats, indent=2), encoding="utf-8")
    record_artifact(STATS_PATH)
    progress["pseudo_label_stats"] = stats
    save_progress()

    progress["stage"] = "train"
    save_progress()
    model = YOLO(MODEL_NAME)
    results = model.train(
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
        name="yolo26n_det_v2",
        exist_ok=True,
        verbose=True,
    )

    save_dir = Path(getattr(model.trainer, "save_dir", getattr(results, "save_dir", RUNS / "yolo26n_det_v2")))
    weights_dir = save_dir / "weights"
    best_src = weights_dir / "best.pt"
    last_src = weights_dir / "last.pt"
    if not best_src.exists():
        raise FileNotFoundError(f"Missing trained checkpoint: {best_src}")

    best_out = WORKING / "best.pt"
    last_out = WORKING / "last.pt"
    shutil.copy2(best_src, best_out)
    record_artifact(best_out)
    if last_src.exists():
        shutil.copy2(last_src, last_out)
        record_artifact(last_out)

    progress["stage"] = "validate_and_sample"
    save_progress()
    trained = YOLO(str(best_out))
    metrics = trained.val(data=str(data_yaml), imgsz=IMG_SIZE, device=0, verbose=False)
    box = getattr(metrics, "box", None)
    metric_payload = {
        "map50": float(getattr(box, "map50", 0.0)),
        "map50_95": float(getattr(box, "map", 0.0)),
        "precision": float(getattr(box, "mp", 0.0)),
        "recall": float(getattr(box, "mr", 0.0)),
        "fitness": float(metrics.fitness()) if callable(getattr(metrics, "fitness", None)) else float(getattr(metrics, "fitness", 0.0)),
        "save_dir": str(save_dir),
        "best_checkpoint": "best.pt",
        "last_checkpoint": "last.pt" if last_out.exists() else None,
    }
    METRICS_PATH.write_text(json.dumps(metric_payload, indent=2), encoding="utf-8")
    record_artifact(METRICS_PATH)

    QUAL_DIR.mkdir(parents=True, exist_ok=True)
    val_records = [r for r in records if r["split"] == "val"]
    random.seed(SEED)
    sample_records = random.sample(val_records, min(12, len(val_records)))
    pred_rows = []
    annotated = []
    for row in sample_records:
        img = cv2.imread(row["image"])
        if img is None:
            continue
        gt_box = (row["x1"], row["y1"], row["x2"], row["y2"])
        draw_box(img, gt_box, f"GT {row['class']}", (255, 0, 0))

        result = trained.predict(source=row["image"], imgsz=IMG_SIZE, conf=0.25, verbose=False)[0]
        best = None
        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            for pred_box, conf, cls_id in zip(xyxy, confs, classes):
                if best is None or float(conf) > best["confidence"]:
                    best = {"box": pred_box.tolist(), "confidence": float(conf), "class_id": int(cls_id)}

        pred_class = None
        pred_conf = 0.0
        pred_iou = 0.0
        if best:
            pred_box = tuple(int(v) for v in best["box"])
            pred_class = CLASSES[best["class_id"]]
            pred_conf = best["confidence"]
            pred_iou = box_iou(gt_box, pred_box)
            draw_box(img, pred_box, f"P {pred_class} {pred_conf:.2f}", (0, 255, 0))

        out_path = QUAL_DIR / Path(row["image"]).name
        cv2.imwrite(str(out_path), img)
        annotated.append(out_path)
        pred_rows.append({
            "image": Path(row["image"]).name,
            "class": row["class"],
            "gt_area_ratio": row["area_ratio"],
            "pred_class": pred_class or "",
            "pred_conf": pred_conf,
            "pred_iou_vs_pseudo_gt": pred_iou,
        })

    write_csv(PRED_CSV, pred_rows, ["image", "class", "gt_area_ratio", "pred_class", "pred_conf", "pred_iou_vs_pseudo_gt"])
    record_artifact(PRED_CSV)
    grid_path = WORKING / "qualitative_grid.jpg"
    make_grid(annotated, grid_path)
    record_artifact(grid_path)
    zip_path = WORKING / "qualitative_predictions.zip"
    zip_dir(QUAL_DIR, zip_path)
    record_artifact(zip_path)

    for pattern in ["*.png", "*.jpg", "*.csv", "args.yaml", "results.csv"]:
        for src in save_dir.glob(pattern):
            dst = WORKING / src.name
            shutil.copy2(src, dst)
            record_artifact(dst)

    for extra in [WORKING / "yolo26n.pt"]:
        if extra.exists():
            extra.unlink()

    progress["metrics"] = metric_payload
    progress["status"] = "complete"
    progress["stage"] = "done"
    save_progress()
    record_artifact(SUMMARY_PATH)
    print(json.dumps(progress, indent=2))
except Exception as exc:
    progress["status"] = "failed"
    progress["error"] = repr(exc)
    progress["traceback"] = traceback.format_exc()
    save_progress()
    print(json.dumps(progress, indent=2))
