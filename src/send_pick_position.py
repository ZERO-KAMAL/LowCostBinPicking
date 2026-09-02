"""
Send a full pick-and-place sequence to a UR5 with a pneumatic/vacuum gripper
controlled through the robot's digital output.

Sequence (all in the ROBOT BASE frame, metres and radians for URScript):
  1. movej  -> approach pose above (x, y) at APPROACH_Z_MM
  2. movel  -> straight down to PICK_Z_MM
  3. gripper ON  (digital output)
  4. wait GRIP_WAIT_S so the vacuum/pneumatics engage
  5. movel  -> straight back up to APPROACH_Z_MM
  6. movej  -> DROP pose
  7. gripper OFF
  8. movej  -> back to HOME pose

IMPORTANT - COORDINATE FRAME
The x_mm / y_mm you send must already be in the ROBOT BASE frame, not in the
"paper rectangle" frame from calibrate.py. Easiest fix: when running
calibrate.py, jog the robot TCP to each of the 4 calibration corners, read the
robot's X/Y on the pendant (Move tab -> Base), and pass THOSE values as
--world-points. Then the homography maps pixels straight into robot coords.

Before running on the real UR5:
  - Set GRIPPER_DO to the digital output pin your pneumatic valve is wired to
  - Confirm GRIPPER_ACTIVE_HIGH (True = output HIGH closes/activates gripper)
  - Measure APPROACH_Z_MM and PICK_Z_MM with the robot jogged manually
  - Set DROP_POSE and HOME_POSE to safe, collision-free poses you tested by jogging
  - Keep velocity/acceleration LOW and stay next to the emergency stop
"""

import socket
import time

HOST = "192.168.1.10"     # <-- change to your UR5's IP (pendant: Settings > Network)
PORT = 30002              # URScript secondary interface

# ---- Heights (mm, robot base frame). Measure these by jogging the robot. ----
APPROACH_Z_MM = -282       # safe height above the bin, must clear the tallest pile
PICK_Z_MM = 20            # height where the gripper touches objects on the bin floor

# ---- Gripper orientation (rotation vector, radians). Gripper pointing down. ----
GRIPPER_RX = 3.14
GRIPPER_RY = 0.0

# ---- Pneumatic / vacuum gripper on digital I/O ----
GRIPPER_DO = 0            # digital output pin number wired to the valve
GRIPPER_ACTIVE_HIGH = True  # True: output HIGH = grip. False: output LOW = grip.
GRIP_WAIT_S = 0.6         # seconds to wait for vacuum/pneumatic to engage
RELEASE_WAIT_S = 0.4      # seconds to wait after releasing

# ---- Fixed poses (metres + radians, robot base frame). SET THESE BY JOGGING. ----
DROP_POSE = [0.30, -0.30, 0.15, GRIPPER_RX, GRIPPER_RY, 0.0]
HOME_POSE = [0.30, 0.00, 0.30, GRIPPER_RX, GRIPPER_RY, 0.0]

# ---- Motion speeds. Keep LOW for first real tests. ----
JOINT_ACC = 0.5
JOINT_VEL = 0.2
LINEAR_ACC = 0.3
LINEAR_VEL = 0.05         # m/s for the vertical descend/ascend


def _pose(x, y, z, rx, ry, rz):
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def send_pick_position(x_mm, y_mm, rx=GRIPPER_RX, ry=GRIPPER_RY, rz=0.0,
                        acceleration=JOINT_ACC, velocity=JOINT_VEL):
    """
    Full pick-and-place at (x_mm, y_mm) with gripper rotation rz (radians).
    Signature matches the call in live_bin_picking.py (no z_mm argument).
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    z_approach = APPROACH_Z_MM / 1000.0
    z_pick = PICK_Z_MM / 1000.0

    grip_on = "True" if GRIPPER_ACTIVE_HIGH else "False"
    grip_off = "False" if GRIPPER_ACTIVE_HIGH else "True"

    approach = _pose(x_m, y_m, z_approach, rx, ry, rz)
    pick = _pose(x_m, y_m, z_pick, rx, ry, rz)
    drop = _pose(*DROP_POSE)
    home = _pose(*HOME_POSE)

    urscript_command = (
        "def pick_and_place():\n"
        f"  set_digital_out({GRIPPER_DO}, {grip_off})\n"
        f"  movej({approach}, a={acceleration}, v={velocity})\n"
        f"  movel({pick}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f"  set_digital_out({GRIPPER_DO}, {grip_on})\n"
        f"  sleep({GRIP_WAIT_S})\n"
        f"  movel({approach}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f"  movej({drop}, a={acceleration}, v={velocity})\n"
        f"  set_digital_out({GRIPPER_DO}, {grip_off})\n"
        f"  sleep({RELEASE_WAIT_S})\n"
        f"  movej({home}, a={acceleration}, v={velocity})\n"
        "end\n"
    )

    print(f"Pick at X={x_mm:.1f}mm Y={y_mm:.1f}mm rz={rz:.3f}rad "
          f"(approach Z={APPROACH_Z_MM}mm, pick Z={PICK_Z_MM}mm)")

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((HOST, PORT))
        s.sendall(urscript_command.encode("utf-8"))
        print("Command sent. Watch the robot - stay next to the emergency stop.")
        time.sleep(1)
        s.close()
    except Exception as e:
        print(f"ERROR: {e}")


if __name__ == "__main__":
    # Manual test with example values (robot base frame, mm).
    send_pick_position(x_mm=300, y_mm=0, rz=0.0)
