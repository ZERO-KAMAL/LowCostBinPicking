# Low-Cost 2D Bin Picking

A low-cost machine vision system for bin picking using a normal webcam.

It finds objects in a bin, works out each object's center and rotation angle,
picks the best one to grab, and outputs real-world coordinates a robot could use.

**How it works:** Webcam image -> YOLO segmentation (finds each object) ->
PCA (center + angle) -> optional depth (which object is on top) -> best pick ->
real-world mm coordinates.

---

## 1. What you need

- A computer (Mac, Windows, or Linux)
- A webcam, fixed and pointing straight down at a bin/box
- Python 3.10 or newer

---

## 2. Install (one time)

Open a terminal inside the project folder, then:

**Mac / Linux**
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**Windows**
```bash
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

You must run the `activate` line every time you open a new terminal.
You will know it worked when you see `(venv)` at the start of the line.

---

## 3. The full workflow

```text
1. Capture images   ->  take photos of objects in the bin
2. Label images     ->  mark each object (in Roboflow)
3. Train            ->  teach YOLO to find your object
4. Run              ->  live detection + best pick
5. Evaluate         ->  get accuracy numbers
```

---

## 4. Capture images

Put objects in the bin and run:

```bash
python src/capture_images.py --config configs/config.yaml --out data/raw --prefix object
```

Controls: `SPACE` = save photo, `+` / `-` = zoom, `q` = quit.

Take about 150-200 photos. Mix it up: single objects, several separated,
some touching, and piled/overlapping. Vary lighting and positions.

Tip: if the camera is wrong, change `camera.index` in `configs/config.yaml`
(try 0, 1, or 2).

---

## 5. Label images (Roboflow)

1. Go to https://roboflow.com and create a project.
2. Type: **Instance Segmentation**. Class name: your object (e.g. `nut`).
3. Upload all photos from `data/raw/`.
4. Draw a polygon around every visible object (each one separately, even if touching).
5. Generate a version (optional: turn on flip/rotate/brightness augmentation).
6. Export as **YOLOv11** (segmentation), download the zip.
7. Unzip it into `data/dataset/` so it has `train/`, `valid/`, and `data.yaml`.

---

## 6. Train

```bash
python src/train_yolo.py --data data/dataset/data.yaml --model yolo11s-seg.pt --epochs 150 --imgsz 640 --name my_model
```

When done, copy your trained model:

```bash
cp runs/segment/my_model/weights/best.pt models/best.pt
```

---

## 7. Run the live system

```bash
python src/live_bin_picking.py --config configs/config.yaml --weights models/best.pt
```

The window shows object masks, center point, angle line, and a red BEST PICK mark.

Controls: `q` = quit, `s` = save screenshot to `outputs/`.

To record a demo video, add `--save outputs/demo.mp4`.

---

## 8. Check accuracy

```bash
python src/evaluate.py --weights models/best.pt --data data/dataset/data.yaml
```

This prints mAP50 (accuracy), precision, recall, and FPS (speed).

---

## 9. Real-world coordinates (calibration)

To turn pixel positions into millimetres (so a robot can use them):

1. Put a sheet of paper with a known rectangle in the bin (e.g. 200 mm x 150 mm).
2. Run:
   ```bash
   python src/calibrate.py --config configs/config.yaml --world-points "0,0;200,0;200,150;0,150"
   ```
3. Click the 4 corners in order, press `s` to save.

After this, the live system prints pick coordinates in mm instead of pixels.

---

## 10. Optional: depth (which object is on top)

Depth helps pick the topmost object in a pile. It is slower, so it is optional.

**Setup (one time):**
```bash
git clone https://github.com/DepthAnything/Depth-Anything-V2.git external/Depth-Anything-V2
pip install -r external/Depth-Anything-V2/requirements.txt
curl -L -o models/depth_anything_v2_vits.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth"
```

**Mac fix (if you get an "MPSFloatType" error):**
In `external/Depth-Anything-V2/depth_anything_v2/dpt.py`, find the `infer_image`
device line and set it to `DEVICE = 'cpu'`.

**Turn it on** in `configs/config.yaml`:
```yaml
depth:
  enabled: true
  method: depth_anything_v2
  model_path: models/depth_anything_v2_vits.pth
  encoder: vits
  every_n_frames: 5
```

**Before running** (every new terminal), tell Python where the depth code is:
```bash
export PYTHONPATH=$PYTHONPATH:external/Depth-Anything-V2
```

To make this permanent (Mac, so you never type it again):
```bash
echo 'export PYTHONPATH=$PYTHONPATH:'"$PWD"'/external/Depth-Anything-V2' >> ~/.zshrc
source ~/.zshrc
```

To turn depth off again, set `enabled: false` in the config.

---

## 11. Optional: classical baseline (for comparison)

A non-AI method (no training) to compare against your YOLO system:

```bash
python src/classical_baseline.py --config configs/config.yaml
```

Add `--invert` if your objects are darker than the background.

---

## 12. Folder layout

```text
bin-picking-2d/
 |- configs/        settings (config.yaml) and dataset info
 |- data/raw/       captured photos
 |- data/dataset/   labeled dataset from Roboflow
 |- models/         trained model (best.pt) + depth weights
 |- outputs/        screenshots and videos
 |- runs/           training results and graphs
 |- src/            all the code
 |- scripts/        shortcut scripts
```

---

## 13. Common problems

| Problem | Fix |
|---|---|
| Camera won't open | Change `camera.index` (0, 1, 2) in config.yaml |
| Mac: camera permission | System Settings > Privacy > Camera > allow Terminal |
| `No module named depth_anything_v2` | Run the `export PYTHONPATH` line (section 10) |
| Mac MPS / MPSFloatType error | Set `DEVICE = 'cpu'` in depth's dpt.py (section 10) |
| Window not responding | Click the window first, then press keys |
| Low FPS with depth on | Increase `every_n_frames` in config (e.g. 5 -> 10) |

---

## 14. Honest limitations

This is a low-cost prototype. It works well for elongated, separated objects
under controlled lighting. It uses one RGB camera, so depth is relative (not exact),
it cannot give a rotation angle for round objects, and it struggles with very
dense piles. It demonstrates feasibility - it does not replace industrial 3D
vision systems.

# when depth is one
# export PYTHONPATH=$PYTHONPATH:external/Depth-Anything-V2
# python src/live_bin_picking.py --config configs/config.yaml --weights models/best.pt
# ./scripts/run_live.sh# LowCostBinPicking
