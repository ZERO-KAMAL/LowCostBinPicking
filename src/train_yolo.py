from __future__ import annotations

import argparse

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="Train YOLO segmentation model for chocolates.")
    parser.add_argument("--data", default="configs/data_chocolate.yaml")
    parser.add_argument("--model", default="yolov8n-seg.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--name", default="chocolate_yolov8n")
    args = parser.parse_args()

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        name=args.name,
        task="segment",
        patience=20,
    )


if __name__ == "__main__":
    main()
