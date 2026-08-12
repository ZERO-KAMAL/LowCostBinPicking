#!/usr/bin/env bash
set -e
python src/evaluate.py --weights runs/segment/chocolate_yolov8n/weights/best.pt --data configs/data_chocolate.yaml
