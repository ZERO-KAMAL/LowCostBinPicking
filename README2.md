# How to run this project (simple guide)

Just follow the
steps in order, top to bottom. Each step tells you exactly what to type or
click.

You will always work from **one terminal window** and **one folder**
(your project folder, e.g. `bin-picking-2d`).

---

## Before you start (one-time setup)

1. **Open a terminal** and go into your project folder:
   ```bash
   cd bin-picking-2d
   ```

2. **Create a virtual environment** (a clean, separate space for this
   project's software so it doesn't interfere with anything else on your
   computer):

   Mac/Linux:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

   Windows:
   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```

   You'll know it worked because your terminal line will now start with
   `(venv)`.

3. **Install the required software packages**:
   ```bash
   pip install -r requirements.txt
   ```
   This may take a few minutes. Let it finish.

You only need to do these 3 steps once. Next time, you only need step 2
(activate the environment) before running anything.

---

## Step 1: Take pictures of your objects (Metal parts, etc.)

This teaches the system what your object looks like.

```bash
python src/capture_images.py --config configs/config.yaml --out data/raw
```

- A camera window will open.
- Press the **spacebar** to save a picture.
- Press **q** to stop when you're done.
- Take around 100 pictures: some with 1 object, some with several, some
  touching or overlapping. Vary the position and lighting.

---

## Step 2: Label your pictures

This is done outside of this project, using a free website called
**Roboflow** (roboflow.com):

1. Upload your pictures from `data/raw`.
2. Choose **Instance Segmentation** as the project type.
3. Draw an outline around every object in every picture.
4. Export the labeled data as **YOLOv8 Segmentation** format.
5. Put the downloaded folder into `data/dataset/` in your project.

---

## Step 3: Train the model

This teaches the computer to recognize your object on its own.

```bash
python src/train_yolo.py --data configs/data.yaml --model yolov8n-seg.pt --epochs 100 --imgsz 640 --name nut_v2
```

- This takes a while (could be minutes to hours depending on your computer).
- When it's done, your trained model will be saved at:
  ```
  runs/segment/nut_v2/weights/best.pt
  ```
- You only need to do this once, unless you want to retrain with new
  pictures later.

---

## Step 4: Calibrate the camera (teach it real-world measurements)

This step tells the system how many millimeters each pixel in the camera
image represents, so it can give real-world positions instead of just
pixels.

1. **Print or draw a rectangle** on paper — for example 200mm wide by
   150mm tall. Measure it with a ruler to be sure.

2. **Place that paper flat inside your bin**, in view of the camera.

3. **Run this command**:
   ```bash
   python src/calibrate.py --config configs/config.yaml
   ```
   If your rectangle is **not** 200mm x 150mm, add your real numbers like
   this (replace 250 and 180 with your actual width and height in mm):
   ```bash
   python src/calibrate.py --config configs/config.yaml --world-points "0,0;250,0;250,180;0,180"
   ```

4. A camera window opens. **Click the 4 corners of your rectangle**, in
   this order:
   - top-left
   - top-right
   - bottom-right
   - bottom-left

5. Once all 4 are clicked, **press the "s" key** to save.

6. The terminal will show you small numbers (errors) for each corner.
   If they're only a few millimeters, it worked. If the numbers look
   huge, press "q", and redo the calibration — click more carefully.

7. This creates a file called `configs/calibration.npz`. You only need to
   redo this if you move the camera.

---

## Step 5: Run the live system

This is the real thing running — detecting objects, picking the best
one, and showing its position.

```bash
python src/live_bin_picking.py --config configs/config.yaml --weights runs/segment/nut_v2/weights/best.pt
```

- A window opens showing the camera feed with colored outlines around
  detected objects and a red circle on the "best pick."
- Press **q** to stop.

---

## Quick troubleshooting

| Problem | What it usually means |
|---|---|
| "Could not open camera" | Wrong camera number in `configs/config.yaml`, or camera is being used by another app. |
| Window doesn't appear at all | You're running this on a computer with no screen attached (e.g. over remote SSH). Run it on a machine with a monitor. |
| Very few or no objects detected | Lighting is too dark, or the model needs more/better training pictures. |
| Calibration error is large (many mm off) | You clicked a corner in the wrong spot, or measured the rectangle incorrectly. Redo Step 4. |

---

## Order to remember

```
Setup (once)
   ↓
Step 1: Take pictures
   ↓
Step 2: Label pictures (Roboflow)
   ↓
Step 3: Train the model
   ↓
Step 4: Calibrate the camera
   ↓
Step 5: Run the live system
```
