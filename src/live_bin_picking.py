from __future__ import annotations

"""Live 2D bin picking with a fully automatic cycle.

Each auto iteration sends one complete cycle (pick -> drop -> home) to the
controller and blocks until the robot reports back, so the loop is self
synchronising: the next detection only happens once the arm is parked at home
and out of the camera view.

Keys:
    p pick once, a toggle auto, h home, d drop, o suction on, f suction off,
    r reset, s screenshot, q quit
"""

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
    send_pick_cycle,
    send_stop,
    send_home,
    send_drop,
    send_vacuum_on,
    send_vacuum_off,
    send_popup,
    APPROACH_Z_MM,
    GRIPPER_RX,
    GRIPPER_RY,
)

# One bad frame should not end a run, but 15 empty in a row means the bin is.
AUTO_EMPTY_FRAMES_NEEDED = 15

# Without this, a part the cup cannot lift would be retried forever.
AUTO_MAX_FAILURES = 3

# 9mm drop + 6 x 15mm = 99mm, still clear of TRANSIT_Z_MM at 120.
MAX_STACK_PARTS = 6
PART_THICKNESS_MM = 15.0


def score_object(confidence: float, area_px: int, depth_value: Optional[float], cfg: Dict[str, Any]) -> float:
    weights = cfg["pick_selection"]
    area_score = min(area_px / 20000.0, 1.0)
    conf_score = confidence
    # Depth Anything V2 gives relative inverse depth, so after normalising,
    # a value near 1.0 means the object is nearest the camera.
    depth_score = 0.5 if depth_value is None else float(np.clip(depth_value, 0, 1))
    return (
        weights["area_weight"] * area_score
        + weights["confidence_weight"] * conf_score
        + weights["depth_weight"] * depth_score
    )


def flush_camera(cap, n: int = 5) -> None:
    """Throw away frames the driver queued while the arm was in the way.
    Without this the next detection runs on a stale picture showing the
    gripper, and the system tries to pick it."""
    for _ in range(n):
        cap.read()


def print_controls() -> None:
    print()
    print("=" * 62)
    print("  CONTROLS - click the video window first, then press a key")
    print("=" * 62)
    print("  p   PICK      pick the object circled in red, then stop")
    print("  a   AUTO      pick repeatedly until the bin is empty")
    print()
    print("  h   HOME      move the arm to the home position")
    print("  d   DROP      move the arm to the drop position")
    print()
    print("  o   SUCTION ON    vacuum on, no movement")
    print("  f   SUCTION OFF   vacuum off with blow-off puff, no movement")
    print()
    print("  r   RESET     stop the arm, release the part, return home,")
    print("                and switch auto mode off")
    print()
    print("  s   SHOT      save a screenshot to outputs/")
    print("  q   QUIT      close the window and exit")
    print("=" * 62)
    print("  Keep a hand on the e-stop. Test with 'p' before using 'a'.")
    print("=" * 62)
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description="Live 2D bin picking with automatic cycling.")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--weights", default=None, help="Path to trained best.pt")
    parser.add_argument("--source", default=None, help="Camera index or image/video path.")
    parser.add_argument("--save", default=None, help="Optional output video path, e.g. outputs/demo.mp4")
    args = parser.parse_args()

    cfg = load_config(args.config)
    weights = args.weights or cfg["model"]["yolo_weights"]
    if not Path(weights).exists() and weights != "yolo11s-seg.pt":
        print(f"WARNING: weights not found: {weights}")
        print("Use --weights runs/segment/<run_name>/weights/best.pt after training.")

    model = YOLO(weights)
    depth = DepthEstimator(
        enabled=cfg["depth"].get("enabled", True),
        method=cfg["depth"].get("method", "none"),
        model_path=cfg["depth"].get("model_path"),
        encoder=cfg["depth"].get("encoder", "vits"),
    )
    depth_every_n = int(cfg["depth"].get("every_n_frames", 5))

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

    # ---- Auto mode state ----
    auto_mode = False
    auto_empty_frames = 0
    auto_failures = 0
    picks_done = 0

    print_controls()

    try:
        while True:
            t0 = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                break
            if cfg["camera"].get("flip_horizontal", False):
                frame = cv2.flip(frame, 1)

            roi_frame, roi_box = apply_roi(frame, cfg["workspace"]["roi"])

            # Depth is expensive: run every Nth frame and reuse the last map.
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
                    x_mm, y_mm = pixel_to_mm(H, p.center_x, p.center_y)
                    cv2.putText(annotated, f"({x_mm:.0f},{y_mm:.0f})mm a={p.angle_deg:.0f}",
                                (int(p.center_x) + 15, int(p.center_y) + 42),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            # ---- HUD ----
            dt = time.perf_counter() - t0
            inst_fps = 1.0 / dt if dt > 0 else 0.0
            fps_smoothed = inst_fps if fps_smoothed is None else 0.9 * fps_smoothed + 0.1 * inst_fps
            cv2.putText(annotated,
                        f"FPS: {fps_smoothed:.1f}  objects: {len(candidates)}  picked: {picks_done}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            if auto_mode:
                cv2.putText(annotated, "AUTO ON - press a to stop", (10, 52),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            legend = "p pick  a auto  h home  d drop  o/f suction  r RESET  s shot  q quit"
            cv2.putText(annotated, legend, (10, annotated.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(annotated, legend, (10, annotated.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            if args.save:
                if writer is None:
                    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(args.save, fourcc, 20.0, (annotated.shape[1], annotated.shape[0]))
                writer.write(annotated)

            cv2.imshow("2D Bin Picking - p pick, a auto, h home, d drop, o/f suction, r reset, q quit", annotated)
            key = cv2.waitKey(1) & 0xFF

            # The window is drawn BEFORE this block, so the operator sees the
            # scene the robot is about to act on. The cycle then blocks until
            # the controller reports back, so commands cannot stack up.
            if auto_mode:
                if H is None:
                    print("Auto mode off: no calibration loaded.")
                    auto_mode = False
                elif candidates:
                    auto_empty_frames = 0
                    if picks_done >= MAX_STACK_PARTS:
                        print(f"[AUTO] Drop stack full after {picks_done} parts.")
                        send_home()
                        send_popup(f"Drop stack full - {picks_done} parts. Empty it.")
                        auto_mode = False
                        frame_idx += 1
                        continue
                    best = max(candidates, key=lambda c: c["score"])
                    p = best["pose"]
                    x_mm, y_mm = pixel_to_mm(H, p.center_x, p.center_y)
                    # A single vacuum cup is symmetric, so aligning the wrist
                    # with the object's long axis gains nothing and only risks
                    # a joint limit. The PCA angle matters for a parallel
                    # gripper or a multi-cup bar.
                    rz = 0
                    # rz = math.radians(p.angle_deg)
                    print(f"[AUTO] pick {best['class']} x={x_mm:.1f}mm y={y_mm:.1f}mm "
                          f"angle={p.angle_deg:.1f}deg")

                    status = send_pick_cycle(
                        x_mm=x_mm,
                        y_mm=y_mm,
                        rx=GRIPPER_RX,
                        ry=GRIPPER_RY,
                        rz=rz,
                        approach_z_mm=APPROACH_Z_MM,
                        drop_z_offset_mm=picks_done * PART_THICKNESS_MM,
                    )

                    if status is None:
                        # We do not know where the arm is, so stop rather than
                        # send another move on top of it.
                        print("[AUTO] No reply from the robot - stopping auto mode.")
                        auto_mode = False
                    elif status == "NO_VACUUM":
                        auto_failures += 1
                        print(f"[AUTO] Grab failed ({auto_failures}/{AUTO_MAX_FAILURES}).")
                        if auto_failures >= AUTO_MAX_FAILURES:
                            print("[AUTO] Too many failed grabs - stopping.")
                            send_popup(f"Stopped: {AUTO_MAX_FAILURES} failed grabs")
                            auto_mode = False
                    else:
                        auto_failures = 0
                        picks_done += 1
                        print(f"[AUTO] Cycle complete. Total picked: {picks_done}")

                    flush_camera(cap, 5)
                else:
                    auto_empty_frames += 1
                    if auto_empty_frames >= AUTO_EMPTY_FRAMES_NEEDED:
                        print(f"[AUTO] Bin appears empty after {picks_done} picks. Stopping.")
                        send_home()
                        send_popup(f"Bin empty - {picks_done} parts picked")
                        auto_mode = False
                        auto_empty_frames = 0

            if key == ord("q"):
                break

            if key == ord("s"):
                Path("outputs").mkdir(exist_ok=True)
                shot = Path("outputs") / f"screenshot_{frame_idx}.jpg"
                cv2.imwrite(str(shot), annotated)
                print(f"Saved {shot}")

            if key == ord("p"):
                if candidates and H is not None:
                    best = max(candidates, key=lambda c: c["score"])
                    p = best["pose"]
                    x_mm, y_mm = pixel_to_mm(H, p.center_x, p.center_y)
                    # rz = math.radians(p.angle_deg)
                    rz = 0
                    print(f"Sending pick: x={x_mm:.1f}mm y={y_mm:.1f}mm angle={p.angle_deg:.1f}deg")
                    send_pick_position(
                        x_mm=x_mm,
                        y_mm=y_mm,
                        rx=GRIPPER_RX,
                        ry=GRIPPER_RY,
                        rz=rz,
                        approach_z_mm=APPROACH_Z_MM,
                    )
                    flush_camera(cap, 5)
                elif H is None:
                    print("Cannot send pick: no calibration loaded. Run calibrate.py first.")
                else:
                    print("Cannot send pick: no object currently detected.")

            if key == ord("r"):
                # Order matters: halt the arm before changing anything, then
                # release whatever is held, then park. Counters are cleared so
                # the next run starts from an empty drop station.
                print("[RESET] Stopping arm, releasing part, returning home...")
                auto_mode = False
                auto_empty_frames = 0
                auto_failures = 0
                picks_done = 0
                try:
                    send_stop()
                    send_vacuum_off()
                    send_home()
                    print("[RESET] Done - arm at home, suction off, auto off.")
                except Exception as exc:
                    print(f"[RESET] Could not complete: {exc}")
                    print("[RESET] If the arm is still moving, use the e-stop.")
                flush_camera(cap, 5)

            if key == ord("o"):
                send_vacuum_on()

            if key == ord("f"):
                send_vacuum_off()

            if key == ord("h"):
                send_home()
                flush_camera(cap, 5)

            if key == ord("d"):
                send_drop()
                flush_camera(cap, 5)

            if key == ord("a"):
                auto_mode = not auto_mode
                auto_empty_frames = 0
                auto_failures = 0
                if auto_mode:
                    print("[AUTO] ON - cycling until the bin is empty. "
                          "Press 'a' again or 'q' to stop. Hand on the e-stop.")
                    # Start from a known pose so the first detection is clean,
                    # and assume the drop station was emptied.
                    picks_done = 0
                    send_home()
                    flush_camera(cap, 5)
                else:
                    print("[AUTO] OFF")

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
