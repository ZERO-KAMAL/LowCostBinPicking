#!/usr/bin/env bash
set -e
python3 src/evaluate.py --weights models/best.pt --data data/dataset/data.yaml