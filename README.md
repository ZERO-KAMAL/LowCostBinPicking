# Low-Cost 2D Bin Picking

A low-cost machine vision system for bin picking using a normal webcam and a
UR10 collaborative robot.

It finds objects in a bin, works out each object's centre,
picks the best one to grab, converts the position into the robot's own
coordinate frame, and runs a complete pick-and-place cycle.

**Vision:** webcam image -> YOLO segmentation (finds each object) -> PCA
(centre) -> optional depth (which object is on top) -> best pick ->
coordinates in mm.

**Robot:** move above the object -> descend until the force sensor feels
contact -> suction on -> lift -> drop -> home -> repeat.

A 2D camera gives X and Y but no height, so the arm finds the object by
touching it. That is what the force-guided descent replaces.

---

## 1. What you need

- A computer (Mac, Windows, or Linux), Python 3.10 or newer
- A webcam, fixed and pointing straight down at a bin
- Optional: a Universal Robots arm with a vacuum gripper, for the full cycle

The vision side runs on its own. The robot sections can be skipped.

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

Run the `activate` line every time you open a new terminal. You will know it
worked when you see `(venv)` at the start of the line.

---

## 3. The full workflow

```text
1. Capture images   ->  photos of objects in the bin
2. Label images     ->  mark each object (in Roboflow)
3. Train            ->  teach YOLO to find your object
4. Calibrate        ->  map image pixels to robot coordinates
5. Set up the robot ->  TCP, payload, home and drop poses
6. Run              ->  live detection and picking
7. Evaluate         ->  accuracy numbers
```

---

## 4. Capture images

Put objects in the bin and run:

```bash
python src/capture_images.py --config configs/config.yaml --out data/raw --prefix object
```

Controls: `SPACE` saves a photo, `+` / `-` zoom, `q` quits.

Take 150-200 photos. Mix single objects, several separated, some touching, and
piled. Vary lighting and position. A model trained only on well-separated
objects fails on the cluttered scenes that bin picking is actually about.

Images are saved already cropped to the workspace region, so training and live
operation see the same field of view.

If the wrong camera opens, change `camera.index` in `configs/config.yaml`
(try 0, 1, or 2).

---

## 5. Label images (Roboflow)

1. Create a project at https://roboflow.com
2. Type: **Instance Segmentation**. One class name for your object.
3. Upload everything from `data/raw/`
4. Draw a polygon around every visible object, each one separately even if they
   touch. For partly hidden objects, outline only the visible part.
5. Generate a version, optionally with flip / rotate / brightness augmentation
6. Export as **YOLOv11 segmentation**, download the zip
7. Unzip into `data/dataset/` so it contains `train/`, `valid/`, and `data.yaml`

Roboflow's automatic labelling speeds this up, but check every mask: it is a
model itself, and it fails on the same touching and occluded cases the trained
model will find hard.

---

## 6. Train

```bash
python src/train_yolo.py --data data/dataset/data.yaml --model yolo11s-seg.pt --epochs 150 --imgsz 640 --name my_model
```

Training stops early if validation performance stops improving. Copy the best
weights when it finishes:

```bash
cp runs/segment/my_model/weights/best.pt models/best.pt
```

`best.pt` is the best-validation checkpoint, not the final epoch. Use it rather
than `last.pt`.

---

## 7. Calibration

Turns pixel positions into millimetres the robot can use.

The reference points come from the robot itself rather than a printed target.
Jogging the arm to the bin corners and reading the pendant gives coordinates
already in the robot's base frame, so no further conversion is needed.

1. Jog the robot to each of the four bin corners. Read X and Y off the pendant
   with **Feature = Base** and write them down, in order.
2. Run, substituting your own four points:
   ```bash
   python src/calibrate.py --config configs/config.yaml \
     --world-points "607.66,103.33;756.36,108.50;756.36,-46.38;609.95,-46.38"
   ```
3. Click the same four corners in the image, in the same order, then press `s`.

The script prints the re-projection error for each point. A few millimetres is
fine.

Redo the calibration if the camera moves, if the bin moves, or if the TCP X/Y
changes. The homography is only valid for the plane it was measured on, so an
object sitting on top of another is reported with a small sideways error.

---

## 8. Robot setup

Skip this section if you are only using the vision side.

**On the pendant**

- `Installation > TCP`: set Z to the distance from the flange face to the tip of
  the suction cup. Check it by jogging the cup onto the bin floor; Z should then
  read close to zero.
- `Installation > Payload`: tool mass and centre of gravity. Without these the
  force reading carries the tool's own weight.
- Switch on **Remote Control**, or the robot refuses programs from the PC.

**In `src/send_pick_position.py`**

| Setting | What it is |
|---|---|
| `HOST` | robot IP |
| `HOME_POSE`, `DROP_POSE` | jog to each station and paste the pendant pose, metres and radians |
| `GRIPPER_RX`, `GRIPPER_RY` | tool orientation; jog until the cup sits flat on a part and read the values |
| `APPROACH_Z_MM` | hover height before the descent |
| `PICK_Z_FLOOR_MM` | hard stop if contact is never felt |
| `TRANSIT_Z_MM` | height for all horizontal travel |
| `PUSH_FORCE_N`, `CONTACT_FORCE_N` | descent force and contact threshold |
| `VACUUM_OUT`, `VACUUM_IN`, `BLOWOFF_OUT` | digital I/O for the gripper |

Test the connection before anything moves:

```bash
python src/test_suction.py        # suction on and off, no movement
python src/move_arm.py            # home, drop, and the waypoints above them
python src/test_force_sensor.py   # descend until contact, with an empty bin
```

---

## 9. Force-guided descent

`force()` on a CB3 controller is the robot's own estimate from joint currents,
not an external sensor. It is noisy and does not rest at zero, so the threshold
has to clear the resting value by a clear margin.

Tune it with an **empty** bin: lower `CONTACT_FORCE_N` until the descent starts
triggering falsely, then go back up one step. `PUSH_FORCE_N` must stay above
`CONTACT_FORCE_N`, or the arm rests against the object without ever generating
enough resistance to trigger.

Keep `DESCEND_SPEED` slow. The force estimate updates too slowly for a fast
move, so a quick descent overshoots the contact point.

`test_force_readout.py` prints the raw force values if you need to see what the
robot is actually measuring.

---

## 10. Run the live system

```bash
python src/live_bin_picking.py --config configs/config.yaml --weights models/best.pt
```

The window shows object masks, centre points, and a red BEST PICK
marker. Click the window first, then press a key.

| Key | Action |
|---|---|
| `p` | pick the marked object, then stop |
| `a` | auto: pick repeatedly until the bin is empty |
| `h` | move to home |
| `d` | move to drop |
| `o` / `f` | suction on / off, no movement |
| `r` | reset: stop, release, return home, clear counters |
| `s` | screenshot to `outputs/` |
| `q` | quit |

Test with `p` before using `a`, and keep a hand on the e-stop.

Add `--save outputs/demo.mp4` to record the session.

**Auto mode stops** when the bin looks empty for 15 consecutive frames, after
three failed grabs in a row, or when the drop stack is full. A message appears
on the pendant in each case.

**Stacking at the drop:** each part is released one part-thickness higher than
the last, since part two lands on part one. Set `PART_THICKNESS_MM` and
`MAX_STACK_PARTS` in `live_bin_picking.py` to match your parts, keeping the
full stack below `TRANSIT_Z_MM`.

---


## 11. Shortcut scripts

The `scripts/` folder wraps the three commands you run most often.

| Script | Runs |
|---|---|
| `./scripts/run_live.sh` | the live system with the usual config and weights |
| `./scripts/calibrate.sh` | calibration with the bin corners hard-coded inside (with four point value which get after jogging the robot)  |
| `./scripts/evaluate.sh` | the accuracy check on the validation set |

`run_live.sh` also sets `PYTHONPATH` for the depth module, so you do not have
to export it by hand.

The bin corner coordinates live inside `calibrate.sh`. Edit them there after
re-measuring, rather than passing them on the command line each time.

---


## 12. Check accuracy

```bash
python src/evaluate.py --weights models/best.pt --data data/dataset/data.yaml
```

Prints mask and box mAP50, mAP50-95, precision, recall, and inference speed.

The speed figure covers segmentation only. The FPS shown in the live window is
lower, because it also includes capture, cropping, pose estimation, scoring and
drawing.

---

## 13. Optional: depth (which object is on top)

Depth helps choose the topmost object in a pile. It is slower, so it is
optional.

**Setup (one time):**
```bash
git clone https://github.com/DepthAnything/Depth-Anything-V2.git external/Depth-Anything-V2
pip install -r external/Depth-Anything-V2/requirements.txt
curl -L -o models/depth_anything_v2_vits.pth \
  "https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth"
```

**Turn it on** in `configs/config.yaml`:
```yaml
depth:
  enabled: true
  method: depth_anything_v2
  model_path: models/depth_anything_v2_vits.pth
  encoder: vits
  every_n_frames: 5
```

**Before running** (every new terminal):
```bash
export PYTHONPATH=$PYTHONPATH:external/Depth-Anything-V2
```

To make that permanent on a Mac:
```bash
echo 'export PYTHONPATH=$PYTHONPATH:'"$PWD"'/external/Depth-Anything-V2' >> ~/.zshrc
source ~/.zshrc
```

---


## 14. Folder layout

```text
bin-picking-2d/
 |- configs/        config.yaml and dataset info
 |- data/raw/       captured photos
 |- data/dataset/   labelled dataset from Roboflow
 |- models/         trained model (best.pt) and depth weights
 |- outputs/        screenshots and videos
 |- runs/           training results and graphs
 |- src/            all the code
 |- scripts/        shortcut scripts
```

---

