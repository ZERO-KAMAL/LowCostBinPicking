"""
Pick motion for a UR5 with a Robotiq 2F gripper and wrist camera.

WHAT THIS DOES
  p -> move above the detected (x, y) at APPROACH_Z_MM, descend straight down
       to PICK_Z_MM, pause, lift back up to APPROACH_Z_MM.
  h -> move to HOME_POSE (parked clear of the camera view).
  d -> move to DROP_POSE.

WHAT THIS DOES NOT DO
  No gripper. The Robotiq 2F is not driven by set_digital_out - it needs the
  Robotiq URCap on the pendant, or a Modbus/RTDE connection.

  No force sensing. The descend goes to a FIXED height, so it works for objects
  lying on the bin floor. Objects sitting on top of a pile are higher than
  PICK_Z_MM, so the gripper would push into them. This is the known limit of a
  2D camera: it gives X and Y, but no height.

COORDINATE FRAME
x_mm / y_mm must be in the ROBOT BASE frame - what calibrate.py produces when
given pendant X/Y readings with Feature = Base.

SAFETY
send_stop() is a convenience, not a safety device. The physical red emergency
stop is the real one - keep a hand near it whenever the arm moves.
"""

import socket
import time

HOST = "192.168.1.10"     # robot IP
PORT = 30002              # URScript secondary interface

# ---- Heights (mm, robot BASE frame). Measured by jogging the robot. ----
APPROACH_Z_MM = 150       # safe hover height above the bin
PICK_Z_MM = 60            # height where the gripper meets an object on the floor

# ---- Tool orientation (rotation vector, radians), measured on the pendant. ----
GRIPPER_RX = 2.866
GRIPPER_RY = -1.214

# Added to the PCA angle before sending as RZ. A parallel gripper closes ACROSS
# the object, so this is usually near 90 deg (1.5708 rad), plus any rotation
# between the camera and robot axes. To measure: hover over an object, jog the
# wrist until the fingers align to grasp it, then subtract the image angle in
# radians from the pendant RZ. Leave at 0.0 until measured.
RZ_OFFSET = 0.0

# ---- Fixed poses (metres + radians, BASE frame), measured by jogging. ----
DROP_POSE = [-0.206, -0.306, 0.150, 2.866, -1.214, 0.069]
HOME_POSE = [-0.2058, -0.1362, 0.150, 2.866, -1.214, 0.069]

# ---- Motion speeds. Keep LOW while testing. ----
JOINT_ACC = 0.3
JOINT_VEL = 0.1
LINEAR_ACC = 0.3
LINEAR_VEL = 0.05

# Seconds to pause at the bottom, so you can see where the gripper landed.
PAUSE_AT_PICK_S = 1.0

STOP_DECEL = 2.0          # rad/s^2 for send_stop()


def _pose(x, y, z, rx, ry, rz):
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def _send(urscript: str, description: str) -> None:
    """Open a socket to the robot, send one URScript block, close."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((HOST, PORT))
        s.sendall(urscript.encode("utf-8"))
        print(f"{description} sent.")
        time.sleep(1)
        s.close()
    except Exception as e:
        print(f"ERROR ({description}): {e}")


def send_stop() -> None:
    """
    Ask the robot to decelerate to a halt.

    CONVENIENCE, not a safety stop. Needs the network up and the robot
    listening. Use the physical emergency stop for anything that matters.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((HOST, PORT))
        s.sendall(f"stopj({STOP_DECEL})\n".encode("utf-8"))
        s.close()
        print("Stop command sent to robot.")
    except Exception as e:
        print(f"Could not send stop: {e}")
        print("If the arm is still moving, use the physical emergency stop.")


def send_home() -> None:
    """Move the arm to HOME_POSE - parked clear of the camera view."""
    script = (
        "def go_home():\n"
        f"  movej({_pose(*HOME_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "  popup(\"At home\", \"Home\", False, False, False)\n"
        "end\n"
    )
    print("Moving to home position.")
    _send(script, "Home command")


def send_drop() -> None:
    """Move the arm to DROP_POSE."""
    script = (
        "def go_drop():\n"
        f"  movej({_pose(*DROP_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "  popup(\"At drop\", \"Drop\", False, False, False)\n"
        "end\n"
    )
    print("Moving to drop position.")
    _send(script, "Drop command")


def send_pick_position(x_mm, y_mm, rx=GRIPPER_RX, ry=GRIPPER_RY, rz=0.0,
                       acceleration=JOINT_ACC, velocity=JOINT_VEL,
                       approach_z_mm=APPROACH_Z_MM, **_ignored):
    """
    Move above (x_mm, y_mm), descend straight down to PICK_Z_MM, pause, lift.

    movej for the approach (fastest path), movel for the vertical moves so the
    gripper travels straight down and up rather than sweeping sideways into
    neighbouring objects.
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    z_approach = approach_z_mm / 1000.0
    z_pick = PICK_Z_MM / 1000.0
    rz_cmd = rz + RZ_OFFSET

    approach = _pose(x_m, y_m, z_approach, rx, ry, rz_cmd)
    pick = _pose(x_m, y_m, z_pick, rx, ry, rz_cmd)

    script = (
        "def pick_move():\n"
        f"  movej({approach}, a={acceleration}, v={velocity})\n"
        f"  movel({pick}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f"  sleep({PAUSE_AT_PICK_S})\n"
        f"  movel({approach}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        "  popup(\"Pick move done\", \"Pick\", False, False, False)\n"
        "end\n"
    )

    print(f"PICK at X={x_mm:.1f}mm Y={y_mm:.1f}mm  "
          f"approach Z={approach_z_mm}mm -> pick Z={PICK_Z_MM}mm  "
          f"rz={rz_cmd:.3f}rad. No gripper.")

    _send(script, "Pick command")
    print("Watch the arm - hand on the emergency stop.")


if __name__ == "__main__":
    # Manual test with a coordinate inside the bin, robot base mm.
    try:
        send_pick_position(x_mm=-450.0, y_mm=-165.0, rz=0.069)
    except KeyboardInterrupt:
        print("\nInterrupted - sending stop to robot.")
        send_stop()
