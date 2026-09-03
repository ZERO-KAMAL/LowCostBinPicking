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
     Different stopping height = the force sensor is working.

Hand on the e-stop both times.
"""

import socket
import time

HOST = "192.168.1.10"
PORT = 30002

FEEDBACK_PORT = 30010
PC_IP = None  # set manually if auto-detect grabs the wrong network adapter

# ---- EDIT THESE for your test ----
X_MM = -450.0
Y_MM = -165.0
APPROACH_Z_MM = 150      # safe hover height, mm, robot base frame
PICK_Z_FLOOR_MM = 25     # lowest Z allowed - measure this on the pendant first
CONTACT_FORCE_N = 8.0   # start here, adjust after watching it run
DESCEND_SPEED = 0.02     # m/s - keep slow
PUSH_FORCE_N = 15.0
DESCEND_TIMEOUT_S = 20

GRIPPER_RX = 2.866
GRIPPER_RY = -1.214
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
        "  zero_ftsensor()\n"
        "  contact = False\n"
        "  hit_floor = False\n"
        "  t_start = 0.0\n"
        "  final_z = 0.0\n"
        f"  force_mode(p[0,0,0,0,0,0], [0,0,1,0,0,0], [0,0,-{PUSH_FORCE_N},0,0,0], 2, "
        f"[0.05,0.05,{DESCEND_SPEED},0.17,0.17,0.17])\n"
        "  while (contact == False and hit_floor == False):\n"
        "    cur = get_actual_tcp_pose()\n"
        f"    if (cur[2] <= {z_floor:.4f}):\n"
        "      hit_floor = True\n"
        "    end\n"
        f"    if (force() > {CONTACT_FORCE_N}):\n"
        "      contact = True\n"
        "    end\n"
        "    t_start = t_start + 0.008\n"
        f"    if (t_start > {DESCEND_TIMEOUT_S}):\n"
        "      hit_floor = True\n"
        "    end\n"
        "    sync()\n"
        "  end\n"
        "  end_force_mode()\n"
        "  stopl(1.0)\n"
        "  p_now = get_actual_tcp_pose()\n"
        "  final_z = p_now[2]\n"
        "  result = \"FLOOR\"\n"
        "  if (contact == True):\n"
        "    result = \"CONTACT\"\n"
        "  end\n"
        f'  socket_open("{_local_ip()}", {FEEDBACK_PORT}, "pc")\n'
        # send both the reason it stopped AND the height, so you can compare
        # empty-bin runs against object runs.
        '  socket_send_line(str_cat(result, str_cat(" ", final_z)), "pc")\n'
        '  socket_close("pc")\n'
        "end\n"
    )

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", FEEDBACK_PORT))
    server.listen(1)
    server.settimeout(30.0)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((HOST, PORT))
    s.sendall(script.encode("utf-8"))
    s.close()
    print("Script sent. Watch the arm - hand on the e-stop.")

    try:
        conn, _ = server.accept()
        conn.settimeout(30.0)
        data = conn.recv(64).decode("utf-8", errors="ignore").strip()
        conn.close()
        print(f"\nRESULT: {data}")
        if data.startswith("CONTACT"):
            print("The force sensor detected contact and stopped the descent.")
        elif data.startswith("FLOOR"):
            print("No contact was felt - the arm stopped at the safety floor.")
            print("Expected if the bin is empty. If there IS an object under")
            print("the cup, raise DESCEND_SPEED slightly or lower CONTACT_FORCE_N.")
    except socket.timeout:
        print("TIMEOUT - no reply from the robot. Check the pendant log:")
        print("  the force_mode script may have been rejected.")
    finally:
        server.close()


if __name__ == "__main__":
    test_descent()
