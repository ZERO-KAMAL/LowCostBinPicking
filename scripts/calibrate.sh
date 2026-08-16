#!/usr/bin/env bash 
set -e
WIDTH=${1:-250}
HEIGHT=${2:-180}
python3 src/calibrate.py --config configs/config.yaml --world-points "0,0;${WIDTH},0;${WIDTH},${HEIGHT};0,${HEIGHT}"

# ./scripts/calibrate.sh 200 150