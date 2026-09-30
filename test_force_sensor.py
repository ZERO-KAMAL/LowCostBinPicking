"""
STANDALONE force-sensor test. No camera, no YOLO, no bin picking pipeline.

This does ONE thing: move above a fixed point you type in, descend slowly, and
stop the moment the force sensor feels contact. This is the smallest possible
proof that "approach until contact" works.

HOW TO USE
  1. Put an object under the arm where you know the (x, y) in mm, robot base
     frame. You already have this from your calibration - use the pendant to
     jog above the object and read X, Y off the pose display if unsure.
  2. Edit X_MM, Y_MM below to that position.
  3. Empty bin first. Run this file. Watch it stop at the floor.
  4. Put a book on the bin floor. Run again. It should stop noticeably higher.
     Different stopping height = the force detection is working.
  5. Tune CONTACT_FORCE_N: lower it until an EMPTY bin starts triggering
     falsely, then go back up one step. That is your margin.

This uses the robot's OWN built-in force estimate from joint currents. There
is no external force sensor. The estimate is noisier and slower than a real
load cell, which is why the descent must be slow and the threshold high.

Hand on the e-stop both times.
"""

import socket
import time

HOST = "172.17.128.102"   # UR10 (confirmed)
PORT = 30002

FEEDBACK_PORT = 30010
PC_IP = None  # set manually if auto-detect grabs the wrong network adapter

# ---- EDIT THESE for your test ----
# Centre of the bin, averaged from the four calibration corners measured
# AFTER the TCP was corrected to Z = 300 mm:
#   (607.66, 103.33) (756.36, 108.50) (756.36, -46.38) (609.95, -46.38)
# About 75 mm clear of every wall.
X_MM = 682.6
Y_MM = 29.8
APPROACH_Z_MM = 115      # safe hover height, mm, robot base frame
# RE-MEASURE with the corrected TCP: jog the cup until it just touches the bin
# bottom, read Z off the pendant, and add about 3 mm. With the old TCP this was
# 42 mm out, which is why the arm kept "reaching the floor" in mid-air.
PICK_Z_FLOOR_MM = 5     # lowest Z allowed

# ABSOLUTE threshold. Free-air noise peaked at 37 N on this robot, so the
# trigger must sit clearly above that. Real contact during a CONTINUOUS move
# blows past it almost immediately, because the arm keeps driving into the
# object instead of stopping after 1 mm.
CONTACT_FORCE_N = 60.0

# How hard force_mode pushes down. Must be ABOVE CONTACT_FORCE_N, otherwise the
# force can never reach the trigger. The arm pushes with this until it is
# stopped, which is what makes contact a large, unambiguous signal.
PUSH_FORCE_N = 50.0

# Slow and gentle. The built-in estimate comes from joint currents, so it
# updates too slowly for a fast move: descend quickly and the tool overshoots
# the contact point before the loop notices.
DESCEND_SPEED = 0.02     # m/s, speed cap on the compliant descent
DESCEND_TIMEOUT_S = 20   # give up after this long
STOP_DECEL = 1.2         # m/s^2 on contact (Juha's pendant node: 1200 mm/s^2)

# How long Python waits for the robot to report back. Must comfortably exceed
# the whole descent.
ACCEPT_TIMEOUT_S = 300.0

GRIPPER_RX = 2.234
GRIPPER_RY = -2.238
JOINT_ACC = 0.15
JOINT_VEL = 0.05    # slowed down - Reduced Mode caps allowed speed, and this
                    # avoids tripping it regardless of which mode is active


def _pose(x, y, z, rx, ry, rz) -> str:
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def _local_ip() -> str:
    if PC_IP:
        return PC_IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((HOST, 1))
        return s.getsockname()[0]
    finally:
        s.close()


def test_descent():
    x_m, y_m = X_MM / 1000.0, Y_MM / 1000.0
    approach = _pose(x_m, y_m, APPROACH_Z_MM / 1000.0, GRIPPER_RX, GRIPPER_RY, 0.0)
    z_floor = PICK_Z_FLOOR_MM / 1000.0

    script = (
        "def force_test():\n"
        f"  movej({approach}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "  sleep(0.5)\n"
        "  zero_ftsensor()\n"
        "  contact = False\n"
        "  hit_floor = False\n"
        "  elapsed = 0.0\n"
        f'  socket_open("{_local_ip()}", {FEEDBACK_PORT}, "pc")\n'
        # ---- force_mode descent, SINGLE THREAD -----------------------------
        # A watcher thread cannot work here: CB3 rejects stopl() from one
        # thread while movel runs in another ("another thread is already
        # controlling the robot"). force_mode avoids that entirely - it makes
        # the arm compliant in Z and pushes down with a constant wrench, all
        # from the main thread, so the same thread can watch force() and stop.
        #
        # Pushing continuously is what gives a usable signal. The arm keeps
        # driving into whatever it meets, so force climbs to the push level
        # instead of the 1 mm nudge a stepped descent produced.
        f"  force_mode(p[0,0,0,0,0,0], [0,0,1,0,0,0], [0,0,-{PUSH_FORCE_N},0,0,0], 2, "
        f"[0.05,0.05,{DESCEND_SPEED},0.17,0.17,0.17])\n"
        "  while (contact == False and hit_floor == False):\n"
        "    cur = get_actual_tcp_pose()\n"
        "    f_now = force()\n"
        f"    if (cur[2] <= {z_floor:.4f}):\n"
        "      hit_floor = True\n"
        "    end\n"
        f"    if (f_now > {CONTACT_FORCE_N}):\n"
        "      contact = True\n"
        "    end\n"
        "    elapsed = elapsed + 0.008\n"
        f"    if (elapsed > {DESCEND_TIMEOUT_S}):\n"
        "      hit_floor = True\n"
        "    end\n"
        "    sync()\n"
        "  end\n"
        "  end_force_mode()\n"
        f"  stopl({STOP_DECEL})\n"
        "  p_now = get_actual_tcp_pose()\n"
        "  final_z = p_now[2]\n"
        '  result = "FLOOR"\n'
        "  if (contact == True):\n"
        '    result = "CONTACT"\n'
        "  end\n"
        '  socket_send_line(str_cat(str_cat(result, str_cat(" z=", to_str(final_z))), str_cat(" f=", to_str(f_now))), "pc")\n'
        '  socket_close("pc")\n'
        "end\n"
    )

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", FEEDBACK_PORT))
    server.listen(1)
    # If this timeout is shorter than the descent, Python closes the listener
    # before the robot reports back, and the pendant logs "Not able to open
    # socket pc to host". Keep it generously long.
    server.settimeout(ACCEPT_TIMEOUT_S)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((HOST, PORT))
    s.sendall(script.encode("utf-8"))
    s.close()
    print("Script sent. Watch the arm - hand on the e-stop.")

    try:
        conn, _ = server.accept()
        conn.settimeout(ACCEPT_TIMEOUT_S)
        buf = ""
        data = ""
        while True:
            chunk = conn.recv(512).decode("utf-8", errors="ignore")
            if not chunk:
                break
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                if line.startswith("STEP"):
                    print(f"  {line}")
                else:
                    data = line
            if data:
                break
        conn.close()
        print(f"\nRESULT: {data}")
        if data.startswith("CONTACT"):
            print(f"Contact detected: force exceeded {CONTACT_FORCE_N} N and the")
            print("descent stopped.")
            print("Now try LOWERING CONTACT_FORCE_N (50, 45, 40) and rerun on an")
            print("EMPTY bin each time. Keep the lowest value that still reaches")
            print("the floor with nothing there - that is your noise margin.")
        elif data.startswith("FLOOR"):
            print("No contact was felt - the arm stopped at the safety floor.")
            print("Expected if the bin is empty.")
            print("If there IS an object under the cup, lower CONTACT_FORCE_N.")
    except socket.timeout:
        print(f"TIMEOUT - no reply within {ACCEPT_TIMEOUT_S:.0f}s.")
        print("Check the pendant Log tab. If it says 'Not able to open socket',")
        print("the descent outlasted this timeout - raise ACCEPT_TIMEOUT_S.")
    finally:
        server.close()


if __name__ == "__main__":
    test_descent()
