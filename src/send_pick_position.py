"""
Force-guided pick motion for a UR10 with a vacuum gripper.

The 2D camera gives X and Y but no height, so the arm finds the object by
touching it: it descends in force_mode until the measured force crosses a
threshold, or until a hard Z floor stops it.

Coordinates are in the robot BASE frame, which is what calibrate.py produces
when given pendant X/Y readings with Feature = Base.

send_stop() is a convenience, not a safety device. Use the physical emergency
stop.
"""

from __future__ import annotations

import socket
import time

# ----connection---
HOST = "172.17.128.102"
PORT = 30002              # URScript secondary interface

FEEDBACK_PORT = 30010     # port on THIS PC the robot reports back to
PC_IP = None              # None = auto-detect this PC's IP

# ---- Heights (mm, robot BASE frame) ---------
APPROACH_Z_MM = 115
PICK_Z_FLOOR_MM = -12     # hard stop if contact is never felt
RETREAT_Z_MM = 115

# movej swings the TCP through an arc, so the middle of a horizontal move dips
# lower than its endpoints. Everything travels at this height instead.
TRANSIT_Z_MM = 120

# ---- Force settings --------
# force() is the robot's own estimate from joint currents, not an external
# sensor, so it is noisy and rests around 25 N with this tool. PUSH must exceed
# CONTACT or the arm rests against the object without ever triggering.
PUSH_FORCE_N = 70.0
CONTACT_FORCE_N = 60.0
DESCEND_SPEED = 0.01      # m/s. Faster and the force estimate lags contact.
DESCEND_TIMEOUT_S = 20.0

# ---- Tool orientation (rotation vector, radians) ---------
# The bracket is bent, so a vertical flange does not mean horizontal cups.
# Found by jogging until the cups sat flat on a part.
GRIPPER_RX = 2.248
GRIPPER_RY = -2.154

RZ_OFFSET = 0.0

# ---- Station poses (metres + radians, BASE frame) ---------
DROP_POSE = [0.39318, -0.37440, 0.009, 2.234, -2.238, 0.028]
HOME_POSE = [0.37135, 0.00285, 0.120, 2.235, -2.238, 0.027]

# ---- Vacuum wiring -------------
VACUUM_OUT = 0            # DO0: suction
VACUUM_SETTLE_S = 1.0     # the cup needs this long to seal before lifting
HAS_VACUUM_SENSOR = True
VACUUM_IN = 0             # DI0: high when the cup has grabbed something
HAS_BLOWOFF = True
BLOWOFF_OUT = 1           # DO1: release puff
BLOWOFF_S = 0.3

# ---- Motion speeds -------------
JOINT_ACC = 1.0
JOINT_VEL = 0.5
LINEAR_ACC = 0.8
LINEAR_VEL = 0.3

STOP_DECEL = 2.0


# ------------------------------------------------------------------ helpers
def _pose(x, y, z, rx, ry, rz) -> str:
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def _pose_at(pose_list, z_mm: float) -> str:
    """Same pose at a different height, for the 'above station' waypoints."""
    x, y, _z, rx, ry, rz = pose_list
    return _pose(x, y, z_mm / 1000.0, rx, ry, rz)


def _grab_block(indent: str = "  ") -> str:
    """Switch the vacuum on and wait for the cup to seal before lifting."""
    i = indent
    return (
        f"{i}set_digital_out({VACUUM_OUT}, True)\n"
        f"{i}sleep({VACUUM_SETTLE_S})\n"
    )


def _release_block(indent: str = "  ") -> str:
    """Switch the vacuum off, with an optional blow-off puff if one is fitted."""
    i = indent
    block = f"{i}set_digital_out({VACUUM_OUT}, False)\n"
    if HAS_BLOWOFF:
        block += (
            f"{i}set_digital_out({BLOWOFF_OUT}, True)\n"
            f"{i}sleep({BLOWOFF_S})\n"
            f"{i}set_digital_out({BLOWOFF_OUT}, False)\n"
        )
    else:
        block += f"{i}sleep(0.3)\n"
    return block


def _vacuum_check_block(indent: str = "  ") -> str:
    """Empty string if no sensor is fitted, in which case a failed grab looks
    identical to a successful one."""
    if not HAS_VACUUM_SENSOR:
        return ""
    i = indent
    return (
        f"{i}if (get_digital_in({VACUUM_IN}) == False):\n"
        f'{i}  status = "NO_VACUUM"\n'
        f"{i}  set_digital_out({VACUUM_OUT}, False)\n"
        f"{i}end\n"
    )


def _local_ip() -> str:
    if PC_IP:
        return PC_IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((HOST, 1))
        return s.getsockname()[0]
    finally:
        s.close()


def _send(urscript: str, description: str) -> bool:
    """Open a socket to the robot, send one URScript block, close."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((HOST, PORT))
        s.sendall(urscript.encode("utf-8"))
        time.sleep(0.3)
        s.close()
        print(f"{description} sent.")
        return True
    except Exception as e:
        print(f"ERROR ({description}): {e}")
        return False


def _send_and_wait(urscript: str, description: str, timeout_s: float = 60.0) -> str | None:
    """Send a script ending in socket_send_line() and block until the robot
    reports back, so the auto loop never has to guess with time.sleep()."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(("0.0.0.0", FEEDBACK_PORT))
        server.listen(1)
        server.settimeout(timeout_s)
    except Exception as e:
        print(f"Could not open feedback port {FEEDBACK_PORT}: {e}")
        server.close()
        return None

    if not _send(urscript, description):
        server.close()
        return None

    try:
        conn, _ = server.accept()
        conn.settimeout(timeout_s)
        data = conn.recv(64).decode("utf-8", errors="ignore").strip()
        conn.close()
        print(f"{description}: robot reported '{data}'")
        if "CONTACT" in data:
            print("The force sensor detected contact and stopped the descent.")
        elif "FLOOR" in data:
            print("No contact was felt - the arm stopped at the safety floor.")
        return data or "DONE"
    except socket.timeout:
        print(f"TIMEOUT waiting for the robot to finish {description}.")
        print("The arm may still be moving - check it before continuing.")
        return None
    except Exception as e:
        print(f"Feedback error ({description}): {e}")
        return None
    finally:
        server.close()


# ------------------------------------------------------------ simple motions
def send_stop() -> None:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((HOST, PORT))
        # end_force_mode() first, in case the stop lands mid-descent.
        s.sendall(f"end_force_mode()\nstopj({STOP_DECEL})\n".encode("utf-8"))
        s.close()
        print("Stop command sent to robot.")
    except Exception as e:
        print(f"Could not send stop: {e}")
        print("If the arm is still moving, use the physical emergency stop.")


def send_vacuum_on() -> None:
    """Suction on by hand. No movement."""
    script = (
        "def vac_on():\n"
        + _grab_block()
        + "end\n"
    )
    if _send(script, "vacuum on"):
        print(f"DO{VACUUM_OUT} ON - suction running")


def send_vacuum_off() -> None:
    """Switch the suction OFF by hand, with the blow-off puff. No movement."""
    script = (
        "def vac_off():\n"
        + _release_block()
        + "end\n"
    )
    if _send(script, "vacuum off"):
        print(f"DO{VACUUM_OUT} OFF - released")


def send_home() -> None:
    """Move the arm to HOME_POSE - parked clear of the camera view."""
    script = (
        "def go_home():\n"
        # Build a new pose rather than assigning into an index: PolyScope 3.x
        # rejects cur[2] = value.
        "  cur = get_actual_tcp_pose()\n"
        f"  up = p[cur[0], cur[1], {TRANSIT_Z_MM / 1000.0:.4f}, cur[3], cur[4], cur[5]]\n"
        f"  movel(up, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f"  movej({_pose_at(HOME_POSE, TRANSIT_Z_MM)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        f"  movel({_pose(*HOME_POSE)}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f'  socket_open("{_local_ip()}", {FEEDBACK_PORT}, "pc")\n'
        '  socket_send_line("HOME_OK", "pc")\n'
        '  socket_close("pc")\n'
        "end\n"
    )
    print("Moving to home position.")
    _send_and_wait(script, "Home command", timeout_s=30.0)

def send_popup(message: str, title: str = "Bin picking") -> None:
    """Pendant message box, then halt. The bin-empty decision is made on the
    PC, so without this the operator has no idea why the arm stopped."""
    # Quotes would break the URScript string literal.
    safe = message.replace('"', "'")
    script = (
        "def show_msg():\n"
        f'  popup("{safe}", "{title}", False, False, False)\n'
        "  halt\n"
        "end\n"
    )
    _send(script, "pendant message")
    
def send_drop() -> None:
    """Move the arm to DROP_POSE and release."""
    script = (
        "def go_drop():\n"
        "  cur = get_actual_tcp_pose()\n"
        f"  up = p[cur[0], cur[1], {TRANSIT_Z_MM / 1000.0:.4f}, cur[3], cur[4], cur[5]]\n"
        f"  movel(up, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f"  movej({_pose_at(DROP_POSE, TRANSIT_Z_MM)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        f"  movel({_pose(*DROP_POSE)}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + _release_block()
        + f'  socket_open("{_local_ip()}", {FEEDBACK_PORT}, "pc")\n'
        '  socket_send_line("DROP_OK", "pc")\n'
        '  socket_close("pc")\n'
        "end\n"
    )
    print("Moving to drop position.")
    _send_and_wait(script, "Drop command", timeout_s=30.0)


# --------------------------------------------------- the force-guided pick
def _descent_block(indent: str = "  ") -> str:
    """Compliant descent that stops on contact or at the floor, whichever
    comes first. This replaces the missing Z from the 2D camera.

    On e-Series, tool_contact() would be cleaner. This is CB3.
    """
    i = indent
    z_floor = PICK_Z_FLOOR_MM / 1000.0
    return (
        f"{i}zero_ftsensor()\n"
        f"{i}contact = False\n"
        f"{i}hit_floor = False\n"
        f"{i}t_start = 0.0\n"
        # Task frame = base frame; only Z is compliant.
        f"{i}force_mode(p[0,0,0,0,0,0], [0,0,1,0,0,0], "
        f"[0,0,-{PUSH_FORCE_N},0,0,0], 2, "
        f"[0.05,0.05,{DESCEND_SPEED},0.17,0.17,0.17])\n"
        f"{i}while (contact == False and hit_floor == False):\n"
        f"{i}  cur = get_actual_tcp_pose()\n"
        f"{i}  if (cur[2] <= {z_floor:.4f}):\n"
        f"{i}    hit_floor = True\n"
        f"{i}  end\n"
        f"{i}  if (force() > {CONTACT_FORCE_N}):\n"
        f"{i}    contact = True\n"
        f"{i}  end\n"
        f"{i}  t_start = t_start + get_steptime()\n"
        f"{i}  if (t_start > {DESCEND_TIMEOUT_S}):\n"
        f"{i}    hit_floor = True\n"
        f"{i}  end\n"
        f"{i}  sync()\n"
        f"{i}end\n"
        f"{i}end_force_mode()\n"
        f"{i}stopl(1.0)\n"
    )


def send_pick_position(x_mm, y_mm, rx=GRIPPER_RX, ry=GRIPPER_RY, rz=0.0,
                       acceleration=JOINT_ACC, velocity=JOINT_VEL,
                       approach_z_mm=APPROACH_Z_MM, **_ignored) -> bool:
    """Pick only: descend until contact, grab, lift. Leaves the part held at
    approach height, for manual testing with the 'p' key."""
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    rz_cmd = rz + RZ_OFFSET

    approach = _pose(x_m, y_m, approach_z_mm / 1000.0, rx, ry, rz_cmd)
    retreat = _pose(x_m, y_m, RETREAT_Z_MM / 1000.0, rx, ry, rz_cmd)
    transit_over_pick = _pose(x_m, y_m, TRANSIT_Z_MM / 1000.0, rx, ry, rz_cmd)

    script = (
        "def pick_move():\n"
        f"  movel({transit_over_pick}, a={acceleration}, v={velocity})\n"
        f"  movel({approach}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + _descent_block()
        + _grab_block()
        + f"  movel({retreat}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + f'  socket_open("{_local_ip()}", {FEEDBACK_PORT}, "pc")\n'
        + '  socket_send_line("PICK_OK", "pc")\n'
        + '  socket_close("pc")\n'
        + "end\n"
    )

    print(f"PICK at X={x_mm:.1f}mm Y={y_mm:.1f}mm  "
          f"approach Z={approach_z_mm}mm -> force descent, floor {PICK_Z_FLOOR_MM}mm  "
          f"rz={rz_cmd:.3f}rad.")
    print("Watch the arm - hand on the emergency stop.")
    return _send_and_wait(script, "Pick command", timeout_s=45.0) is not None


def send_pick_cycle(x_mm, y_mm, rx=GRIPPER_RX, ry=GRIPPER_RY, rz=0.0,
                    approach_z_mm=APPROACH_Z_MM, timeout_s=90.0,drop_z_offset_mm=0.0) -> str | None:
    """One complete cycle as a single URScript program, so the arm never
    pauses mid-cycle waiting for the PC.

    Returns "CYCLE_OK CONTACT", "CYCLE_OK FLOOR", "NO_VACUUM", or None on a
    timeout or connection problem, which should be treated as unsafe.
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    rz_cmd = rz + RZ_OFFSET

    approach = _pose(x_m, y_m, approach_z_mm / 1000.0, rx, ry, rz_cmd)
    retreat = _pose(x_m, y_m, RETREAT_Z_MM / 1000.0, rx, ry, rz_cmd)
    # transit_over_pick = _pose(x_m, y_m, TRANSIT_Z_MM / 1000.0, rx, ry, rz_cmd)
    transit_over_pick = _pose(x_m, y_m, TRANSIT_Z_MM / 1000.0, GRIPPER_RX, GRIPPER_RY, 0.0)

    over_drop = _pose_at(DROP_POSE, TRANSIT_Z_MM)
    # Each part lands on the last, so raise the release height. Clamped below
    # transit so a wrong count can never command a drop above travel height.
    drop_z_mm = min(DROP_POSE[2] * 1000.0 + drop_z_offset_mm, TRANSIT_Z_MM - 10)
    drop_pose_str = _pose_at(DROP_POSE, drop_z_mm)
    over_home = _pose_at(HOME_POSE, TRANSIT_Z_MM)

    script = (
        "def pick_cycle():\n"
        '  status = "CYCLE_OK"\n'
        # Climb before travelling over the bin: approaching sideways at hover
        # height would sweep through anything in the way.
        f"  movej({transit_over_pick}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        f"  movel({approach}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + _descent_block()
        + '  if (contact == True):\n'
        + '    status = "CYCLE_OK CONTACT"\n'
        + '  else:\n'
        + '    status = "CYCLE_OK FLOOR"\n'
        + '  end\n'
        + _grab_block()
        # movel, not movej, so the part is lifted straight up rather than
        # dragged sideways through its neighbours.
        + f"  movel({retreat}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + f"  movel({transit_over_pick}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
         + _vacuum_check_block()
        # Nothing held means nothing to release: skip the drop and park at
        # home, and the next detection tries again.
        + '  if (status != "NO_VACUUM"):\n'
        + f"    movej({over_drop}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        + f"    movel({drop_pose_str}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + _release_block("    ")
        + f"    movel({over_drop}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + "  end\n"
        + f"  movej({over_home}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        + f"  movel({_pose(*HOME_POSE)}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + f'  socket_open("{_local_ip()}", {FEEDBACK_PORT}, "pc")\n'
        + '  socket_send_line(status, "pc")\n'
        + '  socket_close("pc")\n'
        + "end\n"
    )

    print(f"[CYCLE] pick X={x_mm:.1f}mm Y={y_mm:.1f}mm rz={rz_cmd:.3f}rad "
          f"-> drop -> home")
    return _send_and_wait(script, "Pick cycle", timeout_s=timeout_s)


if __name__ == "__main__":
    # Run with an EMPTY bin first, so a bad coordinate only meets the floor.
    try:
        send_home()
        send_pick_position(x_mm=-450.0, y_mm=-165.0, rz=0.069)
        send_home()
    except KeyboardInterrupt:
        print("\nInterrupted - sending stop to robot.")
        send_stop()
