#!/usr/bin/env bash 
set -e
WIDTH=${1:-250}
HEIGHT=${2:-180}
# python3 src/calibrate.py --config configs/config.yaml --world-points "0,0;${WIDTH},0;${WIDTH},${HEIGHT};0,${HEIGHT}"
python3 src/calibrate.py --config configs/config.yaml \
  --world-points="-44.63,-532.65;39.75,-553.65;86.89,-425.45;-12.89,-400.90"
# ./scripts/calibrate.sh 200 150