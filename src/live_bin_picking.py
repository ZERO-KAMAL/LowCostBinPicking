from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
from ultralytics import YOLO

from calibrate import load_calibration, pixel_to_mm
from depth_estimator import DepthEstimator, masked_depth_value
from pca_pose import draw_pose, mask_to_pose
from utils import apply_roi, load_config, normalize_depth
from send_pick_position import (
    send_pick_position,
    send_stop,
    send_home,
    send_drop,
    APPROACH_Z_MM,
    GRIPPER_RX,
    GRIPPER_RY,
)


def score_object(confidence: float, area_px: int, depth_value: Optional[float], cfg: Dict[str, Any]) -> float:
    weights = cfg["pick_selection"]
    area_score = min(area_px / 20000.0, 1.0)
    conf_score = confidence
    # Depth Anything V2 outputs relative inverse depth (larger = closer).
    # After normalize_depth, depth_value near 1.0 usually means "on top".
    depth_score = 0.5 if depth_value is None else float(np.clip(depth_value, 0, 1))
    return (
        weights["area_weight"] * area_score
        + weights["confidence_weight"] * conf_score
        + weights["depth_weight"] * depth_score
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Live 2D bin picking: YOLO mask + PCA + optional depth.")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--weights", default=None, help="Path to trained best.pt")
    parser.add_argument("--source", default=None, help="Camera index or image/video path.")
    parser.add_argument("--save", default=None, help="Optional output video path, e.g. outputs/demo.mp4")
    args = parser.parse_args()

    cfg = load_config(args.config)
    weights = args.weights or cfg["model"]["yolo_weights"]
    if not Path(weights).exists() and weights != "yolov8n-seg.pt":
        print(f"WARNING: weights not found: {weights}")
        print("Use --weights runs/segment/chocolate_yolov8n/weights/best.pt after training.")

    model = YOLO(weights)
    depth = DepthEstimator(
        enabled=cfg["depth"].get("enabled", True),
        method=cfg["depth"].get("method", "none"),
        model_path=cfg["depth"].get("model_path"),
        encoder=cfg["depth"].get("encoder", "vits"),
    )
    depth_every_n = int(cfg["depth"].get("every_n_frames", 5))

    # Load pixel->mm calibration if it exists (created by calibrate.py).
    calib_path = cfg.get("calibration", {}).get("path", "configs/calibration.npz")
    H = load_calibration(calib_path)
    if H is not None:
        print(f"Loaded calibration from {calib_path} - pick coords will be in mm.")
    else:
        print("No calibration found - pick coords will be in pixels. Run calibrate.py to enable mm output.")

    source = args.source if args.source is not None else cfg["camera"]["index"]
    try:
        source = int(source)
    except (TypeError, ValueError):
        pass

    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
    if not cap.isOpened():
        raise RuntimeError("Could not open source/camera. Try changing camera.index in configs/config.yaml")

    writer = None
    depth_map: Optional[np.ndarray] = None
    frame_idx = 0
    fps_smoothed: Optional[float] = None

    try:
        while True:
            t0 = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                break
            if cfg["camera"].get("flip_horizontal", False):
                frame = cv2.flip(frame, 1)

            roi_frame, roi_box = apply_roi(frame, cfg["workspace"]["roi"])

            # Depth is expensive: run only every Nth frame and reuse the last map.
            if depth.enabled and frame_idx % depth_every_n == 0:
                raw_depth = depth.predict(roi_frame)
                depth_map = normalize_depth(raw_depth) if raw_depth is not None else None

            result = model.predict(
                roi_frame,
                conf=cfg["model"]["confidence"],
                imgsz=cfg["model"]["image_size"],
                verbose=False,
            )[0]

            annotated = roi_frame.copy()
            candidates: List[Dict[str, Any]] = []

            if result.masks is not None and result.boxes is not None:
                masks = result.masks.data.cpu().numpy()
                boxes = result.boxes
                for i, mask_float in enumerate(masks):
                    mask = (cv2.resize(mask_float, (roi_frame.shape[1], roi_frame.shape[0])) > 0.5).astype(np.uint8)
                    pose = mask_to_pose(mask)
                    if pose is None or pose.area_px < cfg["pick_selection"]["min_mask_area_px"]:
                        continue

                    conf = float(boxes.conf[i].cpu().numpy())
                    cls_id = int(boxes.cls[i].cpu().numpy())
                    cls_name = model.names.get(cls_id, str(cls_id))
                    dval = masked_depth_value(depth_map, mask)
                    score = score_object(conf, pose.area_px, dval, cfg)
                    candidates.append({"pose": pose, "conf": conf, "class": cls_name, "depth": dval, "score": score})

                    colored_mask = np.zeros_like(annotated)
                    colored_mask[:, :, 1] = mask * 120
                    annotated = cv2.addWeighted(annotated, 1.0, colored_mask, 0.45, 0)
                    label = f"{cls_name} {conf:.2f} angle={pose.angle_deg:.0f}"
                    if dval is not None:
                        label += f" z={dval:.2f}"
                    annotated = draw_pose(annotated, pose, label)

            if candidates:
                best = max(candidates, key=lambda c: c["score"])
                p = best["pose"]
                cv2.circle(annotated, (int(p.center_x), int(p.center_y)), 13, (0, 0, 255), 3)
                cv2.putText(annotated, "BEST PICK", (int(p.center_x) + 15, int(p.center_y) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

                if H is not None:
                    # Convert the pick point from pixels to real-world mm for the robot.
                    x_mm, y_mm = pixel_to_mm(H, p.center_x, p.center_y)
                    cv2.putText(annotated, f"({x_mm:.0f},{y_mm:.0f})mm a={p.angle_deg:.0f}",
                                (int(p.center_x) + 15, int(p.center_y) + 42),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                    # Robot-ready line: easy to parse or write to a file/socket.
                    print(f"PICK x={x_mm:.1f}mm y={y_mm:.1f}mm angle={p.angle_deg:.1f}deg "
                          f"class={best['class']} depth={best['depth']} score={best['score']:.2f}")
                else:
                    print(f"Best: {best['class']} x={p.center_x:.0f}px y={p.center_y:.0f}px "
                          f"angle={p.angle_deg:.1f} depth={best['depth']} score={best['score']:.2f}")

            # FPS measurement (exponential moving average for a stable readout).
            dt = time.perf_counter() - t0
            inst_fps = 1.0 / dt if dt > 0 else 0.0
            fps_smoothed = inst_fps if fps_smoothed is None else 0.9 * fps_smoothed + 0.1 * inst_fps
            cv2.putText(annotated, f"FPS: {fps_smoothed:.1f}  objects: {len(candidates)}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

            if args.save:
                if writer is None:
                    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(args.save, fourcc, 20.0, (annotated.shape[1], annotated.shape[0]))
                writer.write(annotated)

            cv2.imshow("2D Bin Picking - q quit, s shot, p pick, h home, d drop", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                Path("outputs").mkdir(exist_ok=True)
                shot = Path("outputs") / f"screenshot_{frame_idx}.jpg"
                cv2.imwrite(str(shot), annotated)
                print(f"Saved {shot}")

            if key == ord("p"):
                # Manual trigger: send the current best pick to the robot.
                # Only fires once per keypress, not every frame, to avoid
                # flooding the robot with move commands.
                if candidates and H is not None:
                    best = max(candidates, key=lambda c: c["score"])
                    p = best["pose"]
                    x_mm, y_mm = pixel_to_mm(H, p.center_x, p.center_y)
                    rz = math.radians(p.angle_deg)
                    print(f"Sending pick to robot: x={x_mm:.1f}mm y={y_mm:.1f}mm angle={p.angle_deg:.1f}deg")
                    send_pick_position(
                        x_mm=x_mm,
                        y_mm=y_mm,
                        rx=GRIPPER_RX,
                        ry=GRIPPER_RY,
                        rz=rz,
                        approach_z_mm=APPROACH_Z_MM,
                    )
                elif H is None:
                    print("Cannot send pick: no calibration loaded. Run calibrate.py first.")
                else:
                    print("Cannot send pick: no object currently detected.")

            if key == ord("h"):
                # Send the arm to the parked home position, clear of the camera.
                send_home()

            if key == ord("d"):
                # Send the arm to the drop position.
                send_drop()

            frame_idx += 1

    except KeyboardInterrupt:
        print("\nInterrupted - sending stop to robot.")
        send_stop()

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
