# How send_pick_position Works — Force Sensor + Robot Movement

## Your Problem — Why You Need Force Sensor

**Vision System (YOLO + Camera):**
- ✓ Tells you WHERE the object is: X = -455 mm, Y = -159 mm (left-right and front-back)
- ✗ CANNOT tell you how HIGH it is: Z = ???

**Example:** Object is in the bin at X=-455, Y=-159. But is it on top (Z=100mm)? Buried under others (Z=50mm)? You don't know.

**Solution:** Use the force sensor to *feel* for it by pressing down gently until something pushes back.

---

## How `send_pick_position` Actually Works

### Step 1: Your Mac Sends Coordinates
```
Python: send_pick_position(x_mm=-455.7, y_mm=-143.3)
```
The robot now knows: "Go to X=-455.7 and Y=-143.3, but I don't know the Z yet."

---

### Step 2: Robot Moves to APPROACH HEIGHT
```
APPROACH_Z_MM = 150  (150 millimeters above the bin)

Robot does: movej(x=-455.7, y=-143.3, z=150, ...)
```

**What it looks like:** Arm swings over to the spot where the object was detected, hovering 150 mm above the bin floor. Safe. Clear. Ready to feel around.

---

### Step 3: Robot Zeros the Force Sensor (Critical!)
```
zero_ftsensor()
```

**Why this matters:**
- FT300 sensor weighs something, hose weighs something
- If we don't zero, the cup's own weight (maybe 0.5 N) counts toward the 8 N threshold
- After zeroing: "Whatever you feel right now = 0 N. Forget the baseline."

**Before zeroing:** Sensor reads 0.4 N (just the cup sitting)  
**After zeroing:** Sensor reads 0.0 N (now it's your reference)

---

### Step 4: Robot Enters FORCE MODE (The Key Part)
```
force_mode(
  frame=p[0,0,0,0,0,0],           # Work in robot base coordinates
  compliant=[0,0,1,0,0,0],        # Only Z axis is soft / compliant
  target_wrench=[0,0,-15,0,0,0],  # Push down with 15 Newtons
  ...
)
```

**What "compliant" means:** 
- X and Y stay locked — the arm won't drift sideways
- Z is compliant — the arm will move down if something resists

**What "-15 Newtons" means:**
- Robot says: "I want to push down with 15 N of force"
- If nothing is there, the arm sinks as fast as DESCEND_SPEED allows
- If something IS there, it stops pushing and lets the object hold it up

---

### Step 5: The Force Sensor Loop (Runs Every 8 Milliseconds)
```
while (contact == False and hit_floor == False):
    # Check current position
    cur = get_actual_tcp_pose()
    
    # Check 1: Have we hit the floor?
    if (cur[2] <= 0.036):           # 0.036 meters = 36 mm
        hit_floor = True
    end
    
    # Check 2: Do we feel contact?
    if (force() > 8.0):             # If force exceeds 8 Newtons
        contact = True
    end
    
    # Check 3: Have we been searching too long?
    if (t_start > 20.0):            # 20 seconds timeout
        hit_floor = True
    end
    
    sync()  # Update sensors, repeat
end
```

**Timeline of a typical descent:**

| Time | Z Height | Fz Reading | What's Happening |
|------|----------|-----------|-----------------|
| 0 ms | 150 mm | 0.0 N | Hovering, searching starts |
| 500 ms | 135 mm | 0.1 N | Sinking slowly, nothing yet |
| 1000 ms | 120 mm | 0.2 N | Still sinking, still nothing |
| 1500 ms | 105 mm | 0.3 N | Getting close to the object |
| 2000 ms | 95 mm | 2.0 N | Touching the object, force rising |
| 2100 ms | 92 mm | 5.0 N | Pressing harder into it |
| 2150 ms | 90 mm | **8.2 N** | **CONTACT! Force crossed 8.0 N threshold** |
| 2160 ms | 89 mm | 9.5 N | Braking starts (arm slowing down) |
| 2200 ms | 89 mm | 8.0 N | Stopped. Holding steady. |

---

## Why These Numbers Matter

### PICK_Z_FLOOR_MM = 36
**Meaning:** The absolute lowest the arm is allowed to go is 36 mm.

**Why:** If the force sensor fails or never triggers, this is your safety floor. It prevents the arm from crashing into the actual bin floor at 32 mm.

**In real runs:**
- Object is on top → touches at ~90 mm (well above floor)
- Nothing in bin → stops at ~36 mm (safety limit)

---

### CONTACT_FORCE_N = 8.0 Newtons
**Meaning:** Once force exceeds 8 N, declare "contact found."

**Why 8 N?**
- Not too low (would trigger on air currents or hose drag: 2-3 N)
- Not too high (wouldn't trigger on light objects: 12+ N)
- About the weight of a 800-gram bag pressing down — firm but not crushing

**On your steel link parts:**
- Object surface is hard metal
- Force rises quickly from 0 to 8 N as cup settles
- Typically reaches 8-10 N, then brakes to hold steady

---

### PUSH_FORCE_N = 15.0 Newtons
**Meaning:** Robot pushes down with 15 N while searching.

**Why higher than CONTACT_FORCE_N?**
- If they're equal (both 8 N), the arm would stop the *moment* it lands and never actually test whether there's solid contact
- With 15 N push target and 8 N contact threshold, the arm sinks into the object slightly, confirming real contact, not just a light touch

**Visual example:**
```
Scenario 1 (wrong): PUSH=8, CONTACT=8
  Object at Z=90mm
  Force climbs: 0 → 3 → 8 → CONTACT TRIGGERED
  But we barely pressed! Might be a light flake, not solid.

Scenario 2 (correct): PUSH=15, CONTACT=8
  Object at Z=90mm
  Force climbs: 0 → 3 → 8 → CONTACT TRIGGERED
  Arm keeps pressing: 8 → 9 → 10 N
  Now sunk 3-4mm into the object. DEFINITELY solid.
```

---

### DESCEND_SPEED = 0.02 m/s (20 mm/second)
**Meaning:** Max speed while sinking in force mode.

**Why so slow?**
- Fast descent = big overshoot once contact is found
- Slow descent = gentle press, stops cleanly
- At 20 mm/s, takes ~4 seconds to sink from 150 mm to 50 mm

**Can you speed it up?**
- Yes, to 0.04 m/s (40 mm/s) — try it, still safe
- Higher than that risks slamming into the part

---

### DESCEND_TIMEOUT_S = 20.0 seconds
**Meaning:** If nothing is found after 20 seconds, give up.

**Why 20 seconds?**
- At 20 mm/s, descending 100 mm takes 5 seconds
- From 150 mm to 35 mm floor = 115 mm = ~6 seconds
- 20 seconds gives plenty of margin, catches hardware hangs

**What it catches:**
- Force sensor wiring broken → never triggers contact → timeout → stop
- Object too light (< 1 N) → sensor never reaches 8 N → timeout → stop

---

## What Happens at the End

### If Contact Was Found:
```
status = "PICK_OK CONTACT"
```
✓ Robot felt something solid  
✓ Python prints: "The force sensor detected contact and stopped the descent."  
✓ Suction (tomorrow) will turn ON

### If Contact Was NOT Found:
```
status = "PICK_OK FLOOR"
```
✗ Arm reached the safety floor without feeling anything  
✗ Python prints: "No contact was felt - the arm stopped at the safety floor."  
✗ Suction (tomorrow) will stay OFF — nothing to grab

---

## The Real-World Picture

**Your vision gave:** X=-455.7 mm, Y=-143.3 mm (detected in the image)

**Your force sensor gave:** Z=~90 mm (found by pressing down)

**Together:** Full 3D position (X, Y, Z) without needing a depth camera. That's the whole point.


TL;DR version:

Vision gives you X, Y — camera sees the object at these coordinates
Robot hovers 150 mm above — safe starting height
Force sensor zeros — baseline is reset to 0
Robot enters "soft mode" in Z only — pushes down gently with 15 N intent
Loop runs every 8 ms checking three things:
---Have we hit the floor at 36 mm? → Stop
---Do we feel 8+ Newtons? → Stop, contact found
---Have 20 seconds passed? → Timeout safety
Arm stops on whichever comes first — usually contact at ~90 mm, sometimes floor at 36 mm
Python receives the answer — "CONTACT" or "FLOOR" and prints it

That's how a 2D camera system finds Z with a force sensor.