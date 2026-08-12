from __future__ import annotations

"""Classical (non-AI) baseline for the thesis comparison.

Pipeline: grayscale -> Otsu threshold -> morphology -> contours -> PCA pose.
No training data needed. Works for separated, high-contrast objects but
degrades with touching/overlapping objects and lighting changes - exactly
the limitation the YOLO segmentation system is meant to solve. Use this to
produce baseline numbers for the thesis experiments chapter.

Usage:
    python src/classical_baseline.py --config configs/config.yaml
    python src/classical_baseline.py --source data/raw/example.jpg
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from pca_pose import draw_pose, mask_to_pose
from utils import apply_roi, load_config


def segment_classical(roi_frame: np.ndarray, invert: bool = False, min_area: int = 400) -> list[np.ndarray]:
    """Return a list of binary masks, one per detected blob."""
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    flag = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    _, binary = cv2.threshold(blur, 0, 255, flag + cv2.THRESH_OTSU)

    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    masks = []
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(mask, [c], -1, 1, thickness=-1)
        masks.append(mask)
    return masks


def main() -> None:
    parser = argparse.ArgumentParser(description="Classical CV baseline (no AI).")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--source", default=None, help="Camera index or image path.")
    parser.add_argument("--invert", action="store_true",
                        help="Use if objects are darker than background (e.g. dark chocolate on light table).")
    args = parser.parse_args()

    cfg = load_config(args.config)
    source = args.source if args.source is not None else cfg["camera"]["index"]

    # Single image mode.
    if isinstance(source, str) and Path(source).is_file():
        frame = cv2.imread(source)
        roi_frame, _ = apply_roi(frame, cfg["workspace"]["roi"])
        annotated = roi_frame.copy()
        for i, mask in enumerate(segment_classical(roi_frame, invert=args.invert,
                                                   min_area=cfg["pick_selection"]["min_mask_area_px"])):
            pose = mask_to_pose(mask)
            if pose:
                annotated = draw_pose(annotated, pose, f"obj{i} angle={pose.angle_deg:.0f}", color=(255, 160, 0))
        out = Path("outputs") / f"classical_{Path(source).stem}.jpg"
        out.parent.mkdir(exist_ok=True)
        cv2.imwrite(str(out), annotated)
        print(f"Saved {out}")
        return

    # Live camera mode.
    try:
        source = int(source)
    except (TypeError, ValueError):
        pass
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    fps_smoothed = None
    while True:
        t0 = time.perf_counter()
        ok, frame = cap.read()
        if not ok:
            break
        roi_frame, _ = apply_roi(frame, cfg["workspace"]["roi"])
        annotated = roi_frame.copy()
        masks = segment_classical(roi_frame, invert=args.invert,
                                  min_area=cfg["pick_selection"]["min_mask_area_px"])
        for i, mask in enumerate(masks):
            pose = mask_to_pose(mask)
            if pose:
                annotated = draw_pose(annotated, pose, f"obj{i} angle={pose.angle_deg:.0f}", color=(255, 160, 0))

        dt = time.perf_counter() - t0
        fps = 1.0 / dt if dt > 0 else 0.0
        fps_smoothed = fps if fps_smoothed is None else 0.9 * fps_smoothed + 0.1 * fps
        cv2.putText(annotated, f"CLASSICAL BASELINE  FPS: {fps_smoothed:.1f}  objects: {len(masks)}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 160, 0), 2)

        cv2.imshow("Classical baseline - q quit", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
