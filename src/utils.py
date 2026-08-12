from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Tuple

import cv2
import numpy as np
import yaml


def load_config(path: str | Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def ensure_dir(path: str | Path) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


def apply_roi(frame: np.ndarray, roi: list[float]) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    h, w = frame.shape[:2]
    x_pct, y_pct, rw_pct, rh_pct = roi
    x = int(x_pct * w)
    y = int(y_pct * h)
    rw = int(rw_pct * w)
    rh = int(rh_pct * h)
    return frame[y:y + rh, x:x + rw], (x, y, rw, rh)


def draw_roi(frame: np.ndarray, roi_box: Tuple[int, int, int, int]) -> np.ndarray:
    x, y, w, h = roi_box
    out = frame.copy()
    cv2.rectangle(out, (x, y), (x + w, y + h), (255, 255, 255), 2)
    cv2.putText(out, "Workspace ROI", (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def normalize_depth(depth: np.ndarray) -> np.ndarray:
    depth = depth.astype(np.float32)
    d_min, d_max = np.nanmin(depth), np.nanmax(depth)
    if d_max - d_min < 1e-6:
        return np.zeros_like(depth, dtype=np.float32)
    return (depth - d_min) / (d_max - d_min)
