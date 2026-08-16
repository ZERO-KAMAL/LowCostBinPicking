"""
Send a specific pick position to URSim (or a real UR robot).

This is the "final step" of your vision pipeline: once your camera + YOLO + PCA
code has calculated the best object to pick (X, Y position and rotation angle),
this script sends that position to the robot so it can move there.

Before running:
1. URSim must be running (Docker) and the pendant open in your browser
2. Robot must be powered ON with brakes released
"""

import socket
import time

HOST = "127.0.0.1"
PORT = 30002

# ---- FIXED VALUES for your setup (adjust once, based on real measurements) ----
FIXED_Z_MM = 400          # constant pick height
GRIPPER_RX = 3.14         # gripper pointing straight down (adjust to your real setup)
GRIPPER_RY = 0.0


def send_pick_position(x_mm, y_mm, z_mm, rx, ry, rz, acceleration=0.5, velocity=0.2):
    """
    Sends a move command to the robot.

    x_mm, y_mm, z_mm: target position in millimeters (converted to meters below)
    rx, ry, rz: target rotation in radians (gripper orientation)
    acceleration, velocity: how fast the robot moves (keep these LOW in simulation
                             and even lower for first real tests, for safety)
    """
    # UR robots expect position in METERS, not millimeters
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    z_m = z_mm / 1000.0

    urscript_command = (
        "def pick_move():\n"
        f"  target_pose = p[{x_m}, {y_m}, {z_m}, {rx}, {ry}, {rz}]\n"
        f"  movej(target_pose, a={acceleration}, v={velocity})\n"
        "  popup(\"Reached pick position\", \"Pick\", False, False, False)\n"
        "end\n"
    )

    print(f"Sending target position: X={x_mm}mm Y={y_mm}mm Z={z_mm}mm angle(Rz)={rz}rad")

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((HOST, PORT))
        s.sendall(urscript_command.encode('utf-8'))
        print("Command sent. Watch the pendant screen — the robot should move.")
        time.sleep(1)
        s.close()
    except Exception as e:
        print(f"ERROR: {e}")


if __name__ == "__main__":
    # EXAMPLE values only — used only when running this file directly for testing.
    # When imported by live_bin_picking.py, this block is skipped, and the real
    # camera-detected X/Y/angle values are used instead.

    example_x_mm = 300
    example_y_mm = 0
    example_z_mm = 400
    example_rz = 0.0

    send_pick_position(
        x_mm=example_x_mm,
        y_mm=example_y_mm,
        z_mm=example_z_mm,
        rx=3.14,
        ry=0.0,
        rz=example_rz,
    )