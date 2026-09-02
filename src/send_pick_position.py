"""
POSITION TEST VERSION - hover only.

The robot moves to the approach height directly above the (x, y) the camera
detected, and stops there. No descend, no gripper, no drop.

send_home() moves the arm to a fixed parked position, clear of the camera view.

The full pick-and-place URScript is kept below, commented out, so it can be
restored once positioning is verified.

COORDINATE FRAME
The x_mm / y_mm sent here must already be in the ROBOT BASE frame. That is what
calibrate.py produces if you passed pendant X/Y readings (Feature = Base) as
--world-points.

SAFETY
send_stop() is a convenience, NOT a safety device. It only works if the script
is running and the network is up. The physical red emergency stop button is the
real safety mechanism - keep a hand near it whenever the arm moves.
"""

import socket
import time

HOST = "192.168.1.10"     # robot IP
PORT = 30002              # URScript secondary interface

# ---- Heights (mm, robot BASE frame). Measured by jogging the robot. ----
APPROACH_Z_MM = 150       # safe hover height above the bin
PICK_Z_MM = 30            # unused in hover mode; kept for the full sequence

# ---- Tool orientation (rotation vector, radians), measured on the pendant. ----
GRIPPER_RX = 2.866
GRIPPER_RY = -1.214

# ---- Robotiq 2F gripper. NOT a digital output - needs URCap or Modbus/RTDE. ----
# The set_digital_out lines in the commented block below will NOT drive a
# Robotiq 2F. Left in place only as a placeholder for a valve-type gripper.
GRIPPER_DO = 0
GRIPPER_ACTIVE_HIGH = True
GRIP_WAIT_S = 0.6
RELEASE_WAIT_S = 0.4

# ---- Fixed poses (metres + radians, BASE frame), measured by jogging. ----
DROP_POSE = [-0.206, -0.306, 0.150, 2.866, -1.214, 0.069]
HOME_POSE = [-0.2058, -0.1362, 0.150, 2.866, -1.214, 0.069]

# ---- Motion speeds. Keep LOW while testing. ----
JOINT_ACC = 0.3
JOINT_VEL = 0.1
LINEAR_ACC = 0.3
LINEAR_VEL = 0.05

# Deceleration used by send_stop(), in rad/s^2.
STOP_DECEL = 2.0


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
    home = _pose(*HOME_POSE)
    script = (
        "def go_home():\n"
        f"  movej({home}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "  popup(\"At home\", \"Home\", False, False, False)\n"
        "end\n"
    )
    print("Moving to home position.")
    _send(script, "Home command")

def send_drop() -> None:
    """Move the arm to DROP_POSE."""
    drop = _pose(*DROP_POSE)
    script = (
        "def go_drop():\n"
        f"  movej({drop}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "  popup(\"At drop\", \"Drop\", False, False, False)\n"
        "end\n"
    )
    print("Moving to drop position.")
    _send(script, "Drop command")


def send_pick_position(x_mm, y_mm, rx=GRIPPER_RX, ry=GRIPPER_RY, rz=0.0,
                       acceleration=JOINT_ACC, velocity=JOINT_VEL,
                       approach_z_mm=APPROACH_Z_MM, **_ignored):
    """
    Hover above (x_mm, y_mm) at approach_z_mm, then stop.

    approach_z_mm defaults to APPROACH_Z_MM but can be passed by the caller.
    Unexpected keyword arguments are ignored so older callers do not crash.
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    z_approach = approach_z_mm / 1000.0

    approach = _pose(x_m, y_m, z_approach, rx, ry, rz)

    urscript_command = (
        "def hover_above():\n"
        f"  movej({approach}, a={acceleration}, v={velocity})\n"
        "  popup(\"Hover position reached\", \"Position test\", False, False, False)\n"
        "end\n"
    )

    # ---- FULL PICK-AND-PLACE (restore once hovering is verified) ----
    # NOTE: set_digital_out does NOT drive a Robotiq 2F. That gripper needs its
    # URCap or a Modbus/RTDE connection. Replace those lines before using this.
    #
    # z_pick = PICK_Z_MM / 1000.0
    # pick = _pose(x_m, y_m, z_pick, rx, ry, rz)
    # drop = _pose(*DROP_POSE)
    # home = _pose(*HOME_POSE)
    # grip_on = "True" if GRIPPER_ACTIVE_HIGH else "False"
    # grip_off = "False" if GRIPPER_ACTIVE_HIGH else "True"
    # urscript_command = (
    #     "def pick_and_place():\n"
    #     f"  set_digital_out({GRIPPER_DO}, {grip_off})\n"
    #     f"  movej({approach}, a={acceleration}, v={velocity})\n"
    #     f"  movel({pick}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
    #     f"  set_digital_out({GRIPPER_DO}, {grip_on})\n"
    #     f"  sleep({GRIP_WAIT_S})\n"
    #     f"  movel({approach}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
    #     f"  movej({drop}, a={acceleration}, v={velocity})\n"
    #     f"  set_digital_out({GRIPPER_DO}, {grip_off})\n"
    #     f"  sleep({RELEASE_WAIT_S})\n"
    #     f"  movej({home}, a={acceleration}, v={velocity})\n"
    #     "end\n"
    # )

    print(f"HOVER above X={x_mm:.1f}mm Y={y_mm:.1f}mm at Z={approach_z_mm}mm "
          f"(rz={rz:.3f}rad). No descend, no gripper.")

    _send(urscript_command, "Hover command")
    print("Watch the arm - hand on the emergency stop.")


if __name__ == "__main__":
    # Manual test. Coordinate inside the bin, robot base mm.
    try:
        send_pick_position(x_mm=-450.0, y_mm=-165.0, rz=0.069)
    except KeyboardInterrupt:
        print("\nInterrupted - sending stop to robot.")
        send_stop()
