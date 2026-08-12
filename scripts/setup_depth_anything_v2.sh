#!/usr/bin/env bash
set -e
# Optional setup. Run only after YOLO + PCA is working.
# This downloads the Depth Anything V2 source code folder.
git clone https://github.com/DepthAnything/Depth-Anything-V2.git external/Depth-Anything-V2 || true
mkdir -p models
cat <<'MSG'
Now download one Depth Anything V2 checkpoint from the official repository page.
Example target path:
models/depth_anything_v2_vits.pth

Then edit configs/config.yaml:
depth:
  enabled: true
  method: depth_anything_v2
MSG
