#!/usr/bin/env bash
set -e
python src/train_yolo.py --data configs/data_chocolate.yaml --model yolov8n-seg.pt --epochs 100 --imgsz 640 --name chocolate_yolov8n
