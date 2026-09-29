from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np


class DepthEstimator:
    """Optional depth wrapper.

    Default mode returns None, so you can first complete YOLO + PCA.
    Depth Anything V2 can be connected later after installing its repository and weights.
    """

    def __init__(self, enabled: bool = False, method: str = "none", model_path: str | None = None, encoder: str = "vits"):
        self.enabled = enabled
        self.method = method
        self.model_path = model_path
        self.encoder = encoder
        self.model = None

        if self.enabled and self.method == "depth_anything_v2":
            self._load_depth_anything_v2()

    def _load_depth_anything_v2(self) -> None:
        try:
            import torch
            from depth_anything_v2.dpt import DepthAnythingV2
        except Exception as exc:
            raise RuntimeError(
                "Depth Anything V2 is not installed. Follow README optional depth setup first."
            ) from exc

        if not self.model_path or not Path(self.model_path).exists():
            raise FileNotFoundError(f"Depth model weights not found: {self.model_path}")

        model_configs = {
            "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
            "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
            "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
        }
        config = model_configs.get(self.encoder, model_configs["vits"])
        # self.device = "cuda" if torch.cuda.is_available() else "cpu"
        # self.device = "cpu"
        self.device = "mps" if torch.backends.mps.is_available() else "cpu"
        self.model = DepthAnythingV2(**config)
        self.model.load_state_dict(torch.load(self.model_path, map_location="cpu"))
        self.model = self.model.to(self.device).eval()

    def predict(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        if not self.enabled or self.method == "none":
            return None
        if self.method != "depth_anything_v2":
            return None
        depth = self.model.infer_image(frame_bgr)
        return depth.astype(np.float32)


def masked_depth_value(depth_map: Optional[np.ndarray], mask: np.ndarray) -> Optional[float]:
    if depth_map is None:
        return None
    if depth_map.shape[:2] != mask.shape[:2]:
        depth_map = cv2.resize(depth_map, (mask.shape[1], mask.shape[0]))
    # Erode the mask a little so the median is not polluted by blurry
    # depth values at the object boundary (background bleeding).
    eroded = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
    if eroded.sum() > 50:
        mask = eroded
    values = depth_map[mask > 0]
    if values.size == 0:
        return None
    return float(np.median(values))
