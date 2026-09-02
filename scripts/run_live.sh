#!/usr/bin/env bash
set -e
export PYTHONPATH=$PYTHONPATH:$(pwd)/external/Depth-Anything-V2
python3 src/live_bin_picking.py --config configs/config.yaml --weights models/best.pt