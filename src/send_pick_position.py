"""
Force-guided pick motion for a UR10 with a pneumatic SUCTION gripper.

WHAT IS NEW COMPARED TO send_pick_position.py
  1. FORCE-GUIDED DESCENT. Instead of driving to a fixed PICK_Z_MM, the arm
     descends in force_mode until the force sensor feels contact. This is what
     replaces the missing Z from the 2D camera: the robot finds the object
     height by touching it.
  2. HARD Z FLOOR. If contact is never felt, the descent still stops at
     PICK_Z_FLOOR_MM so the arm cannot drive into the bin floor.
  3. DONE HANDSHAKE. The URScript opens a socket back to this PC and sends
     "DONE" when the cycle finishes. Python blocks until it arrives, so the
     auto loop never has to guess with time.sleep().
  4. SUCTION HOOKS. All vacuum lines are written but COMMENTED OUT, so you can
     test the motion and the force descent first. Search for "SUCTION" to
     switch it on later.

COORDINATE FRAME
x_mm / y_mm must be in the ROBOT BASE frame - what calibrate.py produces when
given pendant X/Y readings with Feature = Base.

SAFETY
send_stop() is a convenience, not a safety device. The physical red emergency
stop is the real one - keep a hand near it whenever the arm moves.
Test the first descents with NOTHING in the bin and a low CONTACT_FORCE_N.
"""

from __future__ import annotations

import socket
import time

# ---------------------------------------------------------------- connection
HOST = "172.17.128.101"   # robot IP (UR10, confirmed by ping + suction test)
PORT = 30002              # URScript secondary interface

# Port on THIS PC that the robot reports back to. Must be open in the firewall.
FEEDBACK_PORT = 30010
# Leave as None to auto-detect this PC's IP on the robot network.
PC_IP = None

# ---- Heights (mm, robot BASE frame). Measured by jogging the robot. --------
APPROACH_Z_MM = 115       # safe hover height above the bin
PICK_Z_FLOOR_MM = 25      # RE-MEASURE ON THE UR10: this was the old cell's
                          # floor (~32mm + margin) and will not match.
RETREAT_Z_MM = 115        # height to lift back to right after grabbing

# Height for ALL horizontal travel between stations. Must clear the bin walls,
# the camera mount, and anything else standing in the workspace. movej swings
# the TCP through an arc, so travelling at the same height as the endpoints is
# NOT safe - the middle of the arc dips lower. Go up first, travel, come down.
TRANSIT_Z_MM = 130

# ---- Force settings -------------------------------------------------------
# PUSH_FORCE_N MUST be higher than CONTACT_FORCE_N. If the push force is too
# low, the arm rests gently against a surface without ever generating enough
# resistance to cross the contact threshold - it just sits there until the
# timeout, which looks identical to "no contact" even when it IS touching
# something. Confirmed on hardware: 10/12 never triggered, 15/8 did.
PUSH_FORCE_N = 15.0       # downward force applied while searching
CONTACT_FORCE_N = 8.0     # force that counts as "touched the object"
DESCEND_SPEED = 0.04      # m/s, max speed of the compliant descent. Keep slow.
DESCEND_TIMEOUT_S = 20.0  # confirmed on hardware: 8s was too short for a full
                          # descent from approach height to the floor

# ---- Tool orientation (rotation vector, radians), measured on the pendant. -
GRIPPER_RX = 2.234
GRIPPER_RY = -2.238

# Added to the PCA angle before sending as RZ. For a SUCTION cup the rotation
# barely matters (a round cup is symmetric), so 0.0 is usually fine. Keep the
# offset here in case you fit an oval cup or a two-cup bar later.
RZ_OFFSET = 0.0

# ---- Fixed poses (metres + radians, BASE frame), measured by jogging. ------
# ---- Station poses. RE-TEACH THESE ON THE UR10 -----------------------------
# The values below were taught on the old UR5 cell and will be WRONG here.
# Jog the UR10 to each station, read the TCP pose off the pendant, paste it in.
DROP_POSE = [0.39318, -0.37440, 0.009, 2.234, -2.238, 0.028]
HOME_POSE = [0.37135, 0.00285, 0.130, 2.235, -2.238, 0.027]

# ---- SUCTION wiring -------------------------------------------------------
VACUUM_OUT = 0            # digital output driving the vacuum solenoid valve
VACUUM_SETTLE_S = 1.0     # supervisor: allow one second for vacuum to build
                          # before lifting, otherwise the part is dropped.

# The temporary gripper has no pressure switch and no blow-off valve. Set
# HAS_VACUUM_SENSOR True only once a gripper with feedback is fitted; until
# then the cycle cannot tell a successful grab from an empty one, and
# NO_VACUUM is never reported.
HAS_VACUUM_SENSOR = False
VACUUM_IN = 0             # digital input from the vacuum pressure switch
HAS_BLOWOFF = True        # confirmed: DO1 is the release puff on this gripper
BLOWOFF_OUT = 1           # output for the blow-off / release puff
BLOWOFF_S = 0.3           # how long to puff air to release the part

# ---- Motion speeds. Keep LOW while testing. -------------------------------
JOINT_ACC = 0.5
JOINT_VEL = 0.2
LINEAR_ACC = 0.4
LINEAR_VEL = 0.15

STOP_DECEL = 2.0          # rad/s^2 for send_stop()


# ------------------------------------------------------------------ helpers
def _pose(x, y, z, rx, ry, rz) -> str:
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def _pose_at(pose_list, z_mm: float) -> str:
    """Take a stored pose and return it at a different height.

    Used to build the 'above the drop' and 'above home' waypoints, so the arm
    always travels high and descends vertically instead of cutting corners.
    """
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
    """Ask the pressure switch whether anything was actually grabbed.

    Returns an empty string when no sensor is fitted, in which case the cycle
    always reports success and a failed grab looks identical to a good one.
    """
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
    """Find the IP this PC uses to reach the robot, for the DONE handshake."""
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
    """
    Send a script that ends with socket_send_line(...) and block until the
    robot reports back. Returns the reported status string, or None on failure.

    This is the piece that removes the blind time.sleep() from the auto loop.
    """
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
    """
    Ask the robot to decelerate to a halt.

    CONVENIENCE, not a safety stop. Use the physical emergency stop for
    anything that matters.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((HOST, PORT))
        # end_force_mode() first, in case the stop lands mid-descent and the
        # arm is still compliant.
        s.sendall(f"end_force_mode()\nstopj({STOP_DECEL})\n".encode("utf-8"))
        s.close()
        print("Stop command sent to robot.")
    except Exception as e:
        print(f"Could not send stop: {e}")
        print("If the arm is still moving, use the physical emergency stop.")


def send_home() -> None:
    """Move the arm to HOME_POSE - parked clear of the camera view."""
    script = (
        "def go_home():\n"
        # Lift straight up from wherever the arm currently is, BEFORE any
        # horizontal travel. Build a NEW pose rather than assigning into an
        # index of the existing one - some PolyScope versions reject
        # 'cur[2] = value' with "expected type float but found pose".
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


def send_drop() -> None:
    """Move the arm to DROP_POSE and release."""
    script = (
        "def go_drop():\n"
        "  cur = get_actual_tcp_pose()\n"
        f"  up = p[cur[0], cur[1], {TRANSIT_Z_MM / 1000.0:.4f}, cur[3], cur[4], cur[5]]\n"
        f"  movel(up, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        f"  movej({_pose_at(DROP_POSE, TRANSIT_Z_MM)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        f"  movel({_pose(*DROP_POSE)}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        # ---- SUCTION: release the part -------------------------------------
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
    """
    URScript for the compliant, force-guided descent.

    The arm is put into force_mode with only Z compliant and a gentle downward
    wrench, so it sinks until something pushes back. The loop watches both the
    measured force and the TCP height, so it stops on contact OR at the floor,
    whichever comes first. That is the whole trick that replaces a Z camera.

    NOTE for UR e-Series: tool_contact() is cleaner than this force_mode loop.
    If you are on an e-Series controller, the equivalent is roughly:
        step = tool_contact(direction=get_target_tcp_speed())
        if step > 0: stopl(3.0) ...
    Keep this version for CB3, which is what this UR10 runs. Note that force()
    is the ROBOT'S OWN built-in force estimate from joint currents - no
    external FT sensor is used or needed.
    """
    i = indent
    z_floor = PICK_Z_FLOOR_MM / 1000.0
    return (
        f"{i}zero_ftsensor()\n"
        f"{i}contact = False\n"
        f"{i}hit_floor = False\n"
        f"{i}t_start = 0.0\n"
        # Task frame = base frame. Only Z (index 3 of the vector) is compliant.
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
    """
    PICK ONLY - move above (x, y), descend until contact, grab, lift.

    Use this for manual testing with the 'p' key. It leaves the part held at
    the approach height so you can look at it. For the full automatic cycle
    use send_pick_cycle() instead.
    """
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
        # ---- SUCTION: vacuum on, let the cup seal before lifting ------------
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
                    approach_z_mm=APPROACH_Z_MM, timeout_s=90.0) -> str | None:
    """
    ONE COMPLETE CYCLE, sent as a single URScript program:

        above (x,y) -> force descent -> suction ON -> lift ->
        drop pose -> suction OFF -> home -> report back

    Returns the status string from the robot:
        "CYCLE_OK"   the cycle ran to the end
        "NO_VACUUM"  the cup failed to grab (only once suction is enabled)
        None         timeout or connection problem - treat as unsafe, stop.

    Everything happens inside the robot controller, so the arm never pauses
    mid-cycle waiting for the PC. Python just waits for the report.
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    rz_cmd = rz + RZ_OFFSET

    approach = _pose(x_m, y_m, approach_z_mm / 1000.0, rx, ry, rz_cmd)
    retreat = _pose(x_m, y_m, RETREAT_Z_MM / 1000.0, rx, ry, rz_cmd)
    # transit_over_pick = _pose(x_m, y_m, TRANSIT_Z_MM / 1000.0, rx, ry, rz_cmd)
    transit_over_pick = _pose(x_m, y_m, TRANSIT_Z_MM / 1000.0, GRIPPER_RX, GRIPPER_RY, 0.0)

    # Waypoints directly above the drop and home stations, at transit height.
    over_drop = _pose_at(DROP_POSE, TRANSIT_Z_MM)
    over_home = _pose_at(HOME_POSE, TRANSIT_Z_MM)

    script = (
        "def pick_cycle():\n"
        '  status = "CYCLE_OK"\n'
        # 1. climb to transit height BEFORE travelling over the bin, then
        #    come down to the hover height above the target. Approaching
        #    sideways at hover height would sweep through anything in the way.
        f"  movej({transit_over_pick}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        f"  movel({approach}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        # 2. descend until the force sensor feels the object
        + _descent_block()
        # Fold the descent result into status, so the print above shows it.
        + '  if (contact == True):\n'
        + '    status = "CYCLE_OK CONTACT"\n'
        + '  else:\n'
        + '    status = "CYCLE_OK FLOOR"\n'
        + '  end\n'
        # ---- SUCTION: grab. Vacuum on, then wait for the cup to seal. -------
        + _grab_block()
        # 3. lift STRAIGHT up (movel, not movej) so the part is not dragged
        #    sideways through its neighbours, then continue up to transit.
        + f"  movel({retreat}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        + f"  movel({transit_over_pick}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        # ---- SUCTION: did we actually grab anything? -------------------------
        # The vacuum pressure switch is the only honest answer here. With no
        # switch fitted this block is empty and the loop will happily "pick"
        # empty air forever.
        + _vacuum_check_block()
        # 4. travel HIGH to a point above the drop, then descend vertically.
        #    This is the step that used to be a single movej straight to
        #    DROP_POSE. movej interpolates joints, so the TCP swings through an
        #    arc that can dip into the bin on the way - hence the waypoint.
        + f"  movej({over_drop}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        + f"  movel({_pose(*DROP_POSE)}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        # ---- SUCTION: release the part at the drop station -------------------
        + _release_block()
        # 5. climb back out of the drop station before travelling anywhere.
        + f"  movel({over_drop}, a={LINEAR_ACC}, v={LINEAR_VEL})\n"
        # 6. park at home, clear of the camera, ready for the next detection.
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
    # Manual test. Run this with an EMPTY bin first, so a bad coordinate or a
    # runaway descent only meets the bin floor.
    try:
        send_home()
        send_pick_position(x_mm=-450.0, y_mm=-165.0, rz=0.069)
        send_home()
    except KeyboardInterrupt:
        print("\nInterrupted - sending stop to robot.")
        send_stop()
