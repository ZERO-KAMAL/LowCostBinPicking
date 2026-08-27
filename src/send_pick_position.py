"""
Send a specific pick position to URSim (or a real UR robot / UR5).

This is the "final step" of your vision pipeline: once your camera + YOLO + PCA
code has calculated the best object to pick (X, Y position and rotation angle),
this script sends that position to the robot so it can move there.

Z HEIGHT: the UR5 has a built-in force/torque sensor, so we no longer send a
fixed pick height. Instead the robot moves to a safe APPROACH height above the
bin, then switches to force_mode() and lowers slowly along Z until it feels
contact with the object/bin floor (measured Z force exceeds CONTACT_FORCE_N),
then stops. This adapts to pile height automatically instead of assuming a
constant object height.

Before running:
1. URSim must be running (Docker) and the pendant open in your browser
2. Robot must be powered ON with brakes released
"""

import socket
import time

HOST = "127.0.0.1"
PORT = 30002

# ---- FIXED VALUES for your setup (adjust once, based on real measurements) ----
APPROACH_Z_MM = 400       # safe height above the bin to move to before descending
                          # (must clear the tallest expected pile - not a pick height)
GRIPPER_RX = 3.14         # gripper pointing straight down (adjust to your real setup)
GRIPPER_RY = 0.0

# ---- Force-descent parameters (tune on the real UR5 before trusting this) ----
CONTACT_FORCE_N = 15.0    # stop descending once measured Z force exceeds this (N)
DESCEND_FORCE_N = 10.0    # downward force applied while searching for contact (N)
MAX_DESCEND_SPEED = 0.02  # m/s cap while feeling for contact - keep slow/safe
MAX_DESCEND_MM = 150      # safety cap: give up if no contact within this travel (mm)


def send_pick_position(x_mm, y_mm, rx, ry, rz, acceleration=0.5, velocity=0.2,
                        approach_z_mm=APPROACH_Z_MM):
    """
    Move above the pick (X, Y) at a safe approach height, then use the UR5's
    force/torque sensor to lower onto the object until contact is felt.

    x_mm, y_mm: target position in millimetres (converted to metres below)
    rx, ry, rz: target rotation in radians (gripper orientation)
    approach_z_mm: safe height to move to first, well above the tallest pile
    acceleration, velocity: speed for the approach move (keep LOW for testing)

    Note: there is no z_mm argument anymore - the pick height is found by
    feel via force_mode(), not assumed from vision.
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    z_m = approach_z_mm / 1000.0
    max_descend_m = MAX_DESCEND_MM / 1000.0

    urscript_command = (
        "def pick_move():\n"
        f"  approach_pose = p[{x_m}, {y_m}, {z_m}, {rx}, {ry}, {rz}]\n"
        f"  movej(approach_pose, a={acceleration}, v={velocity})\n"
        "\n"
        "  # Switch to force mode: compliant/free in Z, pushing down gently\n"
        "  # until the F/T sensor feels contact with the object.\n"
        "  force_frame = p[0,0,0,0,0,0]\n"
        "  selection_vector = [0,0,1,0,0,0]\n"
        f"  wrench = [0,0,-{DESCEND_FORCE_N},0,0,0]\n"
        f"  limits = [{MAX_DESCEND_SPEED},{MAX_DESCEND_SPEED},{MAX_DESCEND_SPEED},0.5,0.5,0.5]\n"
        "  force_mode(force_frame, selection_vector, wrench, 2, limits)\n"
        "\n"
        "  start_pose = get_actual_tcp_pose()\n"
        "  contact = False\n"
        "  while not contact:\n"
        "    current_pose = get_actual_tcp_pose()\n"
        "    traveled = start_pose[2] - current_pose[2]\n"
        "    tcp_force = get_tcp_force()\n"
        f"    if norm(tcp_force[2]) > {CONTACT_FORCE_N}:\n"
        "      contact = True\n"
        f"    elif traveled > {max_descend_m}:\n"
        "      contact = True\n"
        "      popup(\"No contact found within safety limit\", \"Pick\", False, False, False)\n"
        "    end\n"
        "    sync()\n"
        "  end\n"
        "  end_force_mode()\n"
        "  popup(\"Contact detected - ready to grip\", \"Pick\", False, False, False)\n"
        "end\n"
    )

    print(f"Sending approach position: X={x_mm}mm Y={y_mm}mm approach_Z={approach_z_mm}mm "
          f"angle(Rz)={rz}rad -- pick height will be found via force sensor contact.")

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((HOST, PORT))
        s.sendall(urscript_command.encode('utf-8'))
        print("Command sent. Watch the pendant screen — the robot should descend until contact.")
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
    example_rz = 0.0

    send_pick_position(
        x_mm=example_x_mm,
        y_mm=example_y_mm,
        rx=3.14,
        ry=0.0,
        rz=example_rz,
    )
