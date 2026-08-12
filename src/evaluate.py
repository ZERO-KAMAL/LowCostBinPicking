from __future__ import annotations

"""Evaluation script for the thesis results chapter.

Reports:
  1. Segmentation accuracy (mAP50, mAP50-95, precision, recall) on the
     validation set, using the standard Ultralytics validation.
  2. Inference speed (mean ms per image and FPS) on validation images.

Usage:
    python src/evaluate.py --weights runs/segment/chocolate_yolov8n/weights/best.pt \
                           --data configs/data_chocolate.yaml
"""

import argparse
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate trained YOLO segmentation model.")
    parser.add_argument("--weights", default="runs/segment/chocolate_yolov8n/weights/best.pt")
    parser.add_argument("--data", default="configs/data_chocolate.yaml")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--images", default="data/dataset/valid/images",
                        help="Folder of images for the speed benchmark.")
    args = parser.parse_args()

    model = YOLO(args.weights)

    print("=" * 60)
    print("1) Accuracy on validation set")
    print("=" * 60)
    metrics = model.val(data=args.data, imgsz=args.imgsz)
    print(f"\nBox  mAP50:    {metrics.box.map50:.3f}")
    print(f"Box  mAP50-95: {metrics.box.map:.3f}")
    print(f"Mask mAP50:    {metrics.seg.map50:.3f}")
    print(f"Mask mAP50-95: {metrics.seg.map:.3f}")
    print(f"Mask precision: {metrics.seg.mp:.3f}")
    print(f"Mask recall:    {metrics.seg.mr:.3f}")

    print("\n" + "=" * 60)
    print("2) Inference speed benchmark")
    print("=" * 60)
    image_paths = sorted(Path(args.images).glob("*.jpg")) + sorted(Path(args.images).glob("*.png"))
    if not image_paths:
        print(f"No images found in {args.images} - skipping speed test.")
        return

    # Warm up (first inference includes model setup overhead).
    warmup = cv2.imread(str(image_paths[0]))
    for _ in range(3):
        model.predict(warmup, imgsz=args.imgsz, verbose=False)

    times = []
    for p in image_paths:
        img = cv2.imread(str(p))
        if img is None:
            continue
        t0 = time.perf_counter()
        model.predict(img, imgsz=args.imgsz, verbose=False)
        times.append(time.perf_counter() - t0)

    mean_ms = 1000.0 * sum(times) / len(times)
    print(f"Images tested: {len(times)}")
    print(f"Mean inference time: {mean_ms:.1f} ms")
    print(f"Throughput: {1000.0 / mean_ms:.1f} FPS")
    print("\nReport these numbers in the thesis results chapter,")
    print("together with the hardware used (CPU/GPU model).")


if __name__ == "__main__":
    main()
