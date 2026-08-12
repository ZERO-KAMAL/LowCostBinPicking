from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class Pose2D:
    center_x: float
    center_y: float
    angle_deg: float
    area_px: int


def mask_to_pose(mask: np.ndarray) -> Optional[Pose2D]:
    """Estimate object center and in-plane angle from a binary mask using PCA."""
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)

    ys, xs = np.where(mask > 0)
    if len(xs) < 20:
        return None

    points = np.column_stack((xs, ys)).astype(np.float32)
    mean, eigenvectors, _ = cv2.PCACompute2(points, mean=None)
    center_x, center_y = mean[0]

    main_axis = eigenvectors[0]
    angle_rad = np.arctan2(main_axis[1], main_axis[0])
    angle_deg = float(np.degrees(angle_rad))

    # PCA axis direction is ambiguous (+v and -v are both valid eigenvectors),
    # which makes the angle jump by 180 deg between frames. For a parallel
    # gripper the grasp is symmetric, so normalize to [-90, 90).
    if angle_deg >= 90.0:
        angle_deg -= 180.0
    elif angle_deg < -90.0:
        angle_deg += 180.0

    return Pose2D(
        center_x=float(center_x),
        center_y=float(center_y),
        angle_deg=angle_deg,
        area_px=int(len(xs)),
    )


def draw_pose(frame: np.ndarray, pose: Pose2D, label: str, color: Tuple[int, int, int] = (0, 255, 0)) -> np.ndarray:
    out = frame.copy()
    cx, cy = int(pose.center_x), int(pose.center_y)
    length = 60
    theta = np.radians(pose.angle_deg)
    x2 = int(cx + length * np.cos(theta))
    y2 = int(cy + length * np.sin(theta))

    cv2.circle(out, (cx, cy), 5, color, -1)
    cv2.line(out, (cx, cy), (x2, y2), color, 3)
    cv2.putText(out, label, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return out
