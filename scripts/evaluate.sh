#!/usr/bin/env bash
set -e
python3 src/evaluate.py --weights runs/segment/nut_v2/weights/best.pt --data data/dataset/data.yaml