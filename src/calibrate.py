from __future__ import annotations

"""Camera calibration for converting pixel coordinates to real-world millimetres.

This uses a HOMOGRAPHY (a flat-plane mapping), which is the right tool for a
fixed top-down camera looking at a flat bin floor. You click 4 known points in
the camera image and tell the script their real-world mm coordinates; the script
computes the pixel->mm transform and saves it. After that, the live system can
report each pick in millimetres, ready to send to a robot.

Why homography (not full chessboard camera calibration)?
- Your camera is fixed and looks straight down at a flat surface.
- You only need (x, y) on that surface, not full 3D lens distortion modelling.
- 4 points and a ruler are enough. Simple, robust, and easy to explain in a thesis.

WORKFLOW
1. Place a ruler/printed grid in the bin. Pick 4 points you can measure, e.g.
   the 4 corners of a 200 x 150 mm rectangle drawn on paper in the bin.
2. Run:  python src/calibrate.py --config configs/config.yaml
3. Click those 4 points in the image (in the order matching --world-points).
4. The script saves configs/calibration.npz (the pixel->mm matrix).

Then live_bin_picking.py automatically loads it and prints mm coordinates.

customize the world points with --world-points, e.g. for a 250x180 mm rectangle:
python src/calibrate.py --config configs/config.yaml --world-points "0,0;250,0;250,180;0,180"
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from utils import apply_roi, load_config


def pixel_to_mm(H: np.ndarray, x_px: float, y_px: float) -> tuple[float, float]:
    """Apply a homography H to a single pixel point, returning (x_mm, y_mm)."""
    pt = np.array([x_px, y_px, 1.0], dtype=np.float64)
    mapped = H @ pt
    mapped /= mapped[2]
    return float(mapped[0]), float(mapped[1])


def load_calibration(path: str | Path) -> np.ndarray | None:
    """Load the saved pixel->mm homography, or None if not calibrated yet."""
    path = Path(path)
    if not path.exists():
        return None
    data = np.load(path)
    return data["H"]


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate pixel->mm using 4 clicked points.")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--out", default="configs/calibration.npz")
    parser.add_argument(
        "--world-points",
        default="0,0;200,0;200,150;0,150",
        help="4 real-world mm points in click order, e.g. '0,0;200,0;200,150;0,150'",
    )
    parser.add_argument("--source", default=None, help="Camera index or image path.")
    args = parser.parse_args()

    cfg = load_config(args.config)

    # Parse the 4 world points (mm).
    world = []
    for pair in args.world_points.split(";"):
        xs, ys = pair.split(",")
        world.append([float(xs), float(ys)])
    world = np.array(world, dtype=np.float64)
    if len(world) != 4:
        raise ValueError("Provide exactly 4 world points.")

    # Grab one frame from the same source and ROI as the live system.
    source = args.source if args.source is not None else cfg["camera"]["index"]
    try:
        source = int(source)
    except (TypeError, ValueError):
        pass

    if isinstance(source, str) and Path(source).is_file():
        frame = cv2.imread(source)
    else:
        cap = cv2.VideoCapture(source)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
        if not cap.isOpened():
            raise RuntimeError("Could not open camera.")
        # Let the camera warm up, then grab a frame.
        for _ in range(10):
            ok, frame = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError("Could not read a frame.")

    if cfg["camera"].get("flip_horizontal", False):
        frame = cv2.flip(frame, 1)

    # Use the SAME ROI crop as the live system so pixel coords match.
    roi_frame, _ = apply_roi(frame, cfg["workspace"]["roi"])

    clicks: list[tuple[int, int]] = []
    display = roi_frame.copy()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))
            cv2.circle(display, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(display, f"{len(clicks)}: {world[len(clicks)-1]}", (x + 8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    win = "Calibration - click 4 points in order, then press s to save, q to quit"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    print("Click the 4 points in the SAME order as your --world-points list.")
    print(f"World points (mm): {world.tolist()}")

    while True:
        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("Cancelled.")
            break
        if key == ord("s"):
            if len(clicks) != 4:
                print(f"Need 4 clicks, have {len(clicks)}.")
                continue
            pixels = np.array(clicks, dtype=np.float64)
            H, _ = cv2.findHomography(pixels, world)
            Path(args.out).parent.mkdir(parents=True, exist_ok=True)
            np.savez(args.out, H=H, pixels=pixels, world=world)
            print(f"Saved calibration to {args.out}")

            # Quick sanity check: re-map the clicked points and show the error.
            print("\nSanity check (clicked pixel -> computed mm vs. true mm):")
            for (px, py), (wx, wy) in zip(clicks, world):
                mx, my = pixel_to_mm(H, px, py)
                err = ((mx - wx) ** 2 + (my - wy) ** 2) ** 0.5
                print(f"  pixel({px},{py}) -> ({mx:6.1f},{my:6.1f}) mm  "
                      f"true({wx:5.1f},{wy:5.1f})  error={err:.1f} mm")
            print("\nIf errors are small (a few mm), calibration is good.")
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
