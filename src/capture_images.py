from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import cv2

from utils import apply_roi, draw_roi, ensure_dir, load_config


def apply_zoom(frame, zoom: float):
    """Digital zoom: crop the center by 1/zoom and scale back to full size."""
    if zoom <= 1.0:
        return frame
    h, w = frame.shape[:2]
    cw, ch = int(w / zoom), int(h / zoom)
    x = (w - cw) // 2
    y = (h - ch) // 2
    cropped = frame[y:y + ch, x:x + cw]
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture dataset images from webcam.")
    parser.add_argument("--config", default="configs/config.yaml")
    parser.add_argument("--out", default="data/raw")
    parser.add_argument("--prefix", default="object")
    args = parser.parse_args()

    cfg = load_config(args.config)
    out_dir = ensure_dir(args.out)

    cam_cfg = cfg["camera"]
    cap = cv2.VideoCapture(cam_cfg["index"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg["height"])

    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try changing camera.index in configs/config.yaml")

    count = len(list(Path(out_dir).glob("*.jpg")))
    zoom = 1.0
    print("Controls: SPACE = save image, + = zoom in, - = zoom out, q = quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if cam_cfg.get("flip_horizontal", False):
            frame = cv2.flip(frame, 1)

        # Apply digital zoom to the whole frame first.
        frame = apply_zoom(frame, zoom)

        roi_frame, roi_box = apply_roi(frame, cfg["workspace"]["roi"])
        preview = draw_roi(frame, roi_box)
        cv2.putText(preview, f"Saved: {count}  Zoom: {zoom:.1f}x", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("Capture dataset - SPACE save, +/- zoom, q quit", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == 32:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = Path(out_dir) / f"{args.prefix}_{timestamp}.jpg"
            # Save only workspace ROI to keep training focused.
            cv2.imwrite(str(path), roi_frame)
            count += 1
            print(f"Saved {path}")
        if key in (ord("+"), ord("=")):   # '=' so you don't need Shift
            zoom = min(zoom + 0.2, 5.0)
        if key in (ord("-"), ord("_")):
            zoom = max(zoom - 0.2, 1.0)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()