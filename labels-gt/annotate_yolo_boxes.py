#!/usr/bin/env python3
import argparse
import json
import random
import shutil
from pathlib import Path

import cv2

CLASSES = ["left-arrow", "right-arrow", "stop-signal"]
CLASS_TO_ID = {name: idx for idx, name in enumerate(CLASSES)}


def repo_root():
    return Path(__file__).resolve().parents[1]


def parse_args():
    root = repo_root()
    default_source = root / "output" / "input"
    default_out = root / "labels-gt" / "dataset"
    parser = argparse.ArgumentParser(
        description="Annotate one YOLO detection bbox per signal image."
    )
    parser.add_argument("--source", type=Path, default=default_source)
    parser.add_argument("--out", type=Path, default=default_out)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--redo", action="store_true", help="Re-annotate existing labels.")
    parser.add_argument("--limit", type=int, default=0, help="Annotate at most N pending images.")
    return parser.parse_args()


def image_paths(source):
    rows = []
    for class_name in CLASSES:
        class_dir = source / class_name
        rows.extend((class_name, path) for path in sorted(class_dir.glob("*.jpg")))
    return rows


def split_map(rows, val_ratio, seed):
    splits = {}
    rng = random.Random(seed)
    for class_name in CLASSES:
        class_rows = [path for cls, path in rows if cls == class_name]
        shuffled = class_rows[:]
        rng.shuffle(shuffled)
        val_count = max(1, round(len(shuffled) * val_ratio)) if shuffled else 0
        val_set = {path for path in shuffled[:val_count]}
        for path in class_rows:
            splits[path] = "val" if path in val_set else "train"
    return splits


def output_name(class_name, image_path):
    return f"{class_name}_{image_path.name}"


def label_path(out, split, class_name, image_path):
    return out / "labels" / split / output_name(class_name, image_path).replace(".jpg", ".txt")


def image_out_path(out, split, class_name, image_path):
    return out / "images" / split / output_name(class_name, image_path)


def yolo_line(class_id, roi, width, height):
    x, y, w, h = roi
    cx = (x + w / 2) / width
    cy = (y + h / 2) / height
    nw = w / width
    nh = h / height
    return f"{class_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n"


def read_existing_box(label_file, image_width, image_height):
    if not label_file.exists():
        return None
    parts = label_file.read_text(encoding="utf-8").strip().split()
    if len(parts) != 5:
        return None
    _, cx, cy, w, h = map(float, parts)
    px = int(round((cx - w / 2) * image_width))
    py = int(round((cy - h / 2) * image_height))
    pw = int(round(w * image_width))
    ph = int(round(h * image_height))
    return max(0, px), max(0, py), max(1, pw), max(1, ph)


def write_data_yaml(out):
    names = "\n".join(f"  {idx}: {name}" for idx, name in enumerate(CLASSES))
    text = (
        f"path: {out.resolve().as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        f"{names}\n"
    )
    (out / "data.yaml").write_text(text, encoding="utf-8")


def write_state(out, total, annotated, skipped):
    payload = {
        "classes": CLASSES,
        "total_images": total,
        "annotated": annotated,
        "skipped_this_run": skipped,
    }
    (out.parent / "annotation_state.json").write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )


def confirm_roi(title, img, roi):
    x, y, w, h = [int(v) for v in roi]
    preview = img.copy()
    cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.putText(
        preview,
        "s/Enter=guardar  r=rehacer  c=saltar",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    while True:
        cv2.imshow(title, preview)
        key = cv2.waitKey(0) & 0xFF
        if key in (13, 32, ord("s")):
            cv2.destroyWindow(title)
            return "save"
        if key == ord("r"):
            cv2.destroyWindow(title)
            return "redo"
        if key == ord("c"):
            cv2.destroyWindow(title)
            return "skip"


def confirm_cancel(title, img):
    preview = img.copy()
    cv2.putText(
        preview,
        "Seleccion cancelada: r=rehacer  c=saltar",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 200, 255),
        2,
        cv2.LINE_AA,
    )
    while True:
        cv2.imshow(title, preview)
        key = cv2.waitKey(0) & 0xFF
        if key == ord("r"):
            cv2.destroyWindow(title)
            return "redo"
        if key == ord("c"):
            cv2.destroyWindow(title)
            return "skip"


def annotate_one(class_name, image_path, split, out, redo):
    label_file = label_path(out, split, class_name, image_path)
    dst_image = image_out_path(out, split, class_name, image_path)
    if label_file.exists() and dst_image.exists() and not redo:
        return "existing"

    img = cv2.imread(str(image_path))
    if img is None:
        return "bad_image"

    height, width = img.shape[:2]
    preview = img.copy()
    existing = read_existing_box(label_file, width, height)
    if existing:
        x, y, w, h = existing
        cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 200, 255), 2)

    title = f"{class_name} | {image_path.name}"
    print(f"\n{title}")
    while True:
        print("Arrastra la caja. Enter/Espacio termina seleccion. c cancela seleccion.")
        roi = cv2.selectROI(title, preview, showCrosshair=True, fromCenter=False)
        cv2.destroyWindow(title)

        x, y, w, h = [int(v) for v in roi]
        if w <= 0 or h <= 0:
            action = confirm_cancel(title, img)
            if action == "skip":
                return "skipped"
            continue

        action = confirm_roi(title, img, roi)
        if action == "redo":
            continue
        if action == "skip":
            return "skipped"
        break

    label_file.parent.mkdir(parents=True, exist_ok=True)
    dst_image.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(image_path, dst_image)
    label_file.write_text(
        yolo_line(CLASS_TO_ID[class_name], (x, y, w, h), width, height),
        encoding="utf-8",
    )
    return "annotated"


def main():
    args = parse_args()
    rows = image_paths(args.source)
    if not rows:
        raise SystemExit(f"No JPG images found under {args.source}")

    splits = split_map(rows, args.val_ratio, args.seed)
    args.out.mkdir(parents=True, exist_ok=True)
    write_data_yaml(args.out)

    annotated = 0
    skipped = 0
    processed = 0
    total = len(rows)
    for idx, (class_name, image_path) in enumerate(rows, start=1):
        if args.limit and processed >= args.limit:
            break
        split = splits[image_path]
        label_file = label_path(args.out, split, class_name, image_path)
        dst_image = image_out_path(args.out, split, class_name, image_path)
        if label_file.exists() and dst_image.exists() and not args.redo:
            annotated += 1
            continue

        print(f"[{idx}/{total}] class={class_name} split={split} image={image_path}")
        status = annotate_one(class_name, image_path, split, args.out, args.redo)
        processed += 1
        if status in {"annotated", "existing"}:
            annotated += 1
        elif status == "skipped":
            skipped += 1
        else:
            print(f"status={status}: {image_path}")
        write_state(args.out, total, annotated, skipped)

    write_state(args.out, total, annotated, skipped)
    print(f"\nDone. Annotated labels: {annotated}/{total}. Skipped this run: {skipped}.")
    print(f"Dataset YAML: {args.out / 'data.yaml'}")


if __name__ == "__main__":
    main()
