"""Minimal move + suction test for the UR10. Nothing imported, nothing shared.

Edit the poses below, run it, press a key.

    1  home                 o  suction ON
    2  drop                 f  suction OFF (with blow-off puff)
    3  above home
    4  above drop
    s  STOP
    q  quit (suction off first)

Keep a hand on the e-stop. Remote Control must be ON on the pendant.

Usage:
    python move_arm.py
"""

import socket

# ---------------------------------------------------------------- connection
HOST = "172.17.128.102"
PORT = 30002

# ---- Poses: x, y, z in METRES, rx, ry, rz in RADIANS -----------------------
# The pendant shows mm and degrees, so divide mm by 1000 when pasting.
HOME_POSE = [0.37135, 0.00285, 0.130, 2.235, -2.238, 0.027]
DROP_POSE = [0.39318, -0.37440, 0.009, 2.234, -2.238, 0.028]

# Height used for the "above" positions, in mm.
TRANSIT_Z_MM = 130

# ---- Suction wiring --------------------------------------------------------
VACUUM_OUT = 0        # DO0 - suction on
BLOWOFF_OUT = 1       # DO1 - release puff
BLOWOFF_S = 0.3       # how long to puff air when releasing

# ---- Speeds. Keep these LOW while testing. ---------------------------------
ACC = 0.3
VEL = 0.1


def pose_str(x, y, z, rx, ry, rz):
    return f"p[{x:.5f},{y:.5f},{z:.5f},{rx:.5f},{ry:.5f},{rz:.5f}]"


def above(base, z_mm):
    """Same x, y and orientation as base, but at height z_mm."""
    x, y, _, rx, ry, rz = base
    return pose_str(x, y, z_mm / 1000.0, rx, ry, rz)


def send(script):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    try:
        s.connect((HOST, PORT))
        s.sendall(script.encode())
    finally:
        s.close()


def move_to(label, p):
    send(f"def m():\n  movej({p}, a={ACC}, v={VEL})\nend\n")
    print(f"  -> {label}  {p}")


def vacuum_on():
    send(f"def v():\n  set_digital_out({VACUUM_OUT}, True)\nend\n")
    print(f"  DO{VACUUM_OUT} ON  - suction running")


def vacuum_off():
    send(
        "def v():\n"
        f"  set_digital_out({VACUUM_OUT}, False)\n"
        f"  set_digital_out({BLOWOFF_OUT}, True)\n"
        f"  sleep({BLOWOFF_S})\n"
        f"  set_digital_out({BLOWOFF_OUT}, False)\n"
        "end\n"
    )
    print(f"  DO{VACUUM_OUT} OFF - DO{BLOWOFF_OUT} puffed to release")


def stop():
    send("def s():\n  stopj(2.0)\nend\n")
    print("  STOP sent")


def main():
    print(f"Robot: {HOST}:{PORT}")
    print(f"HOME  {HOME_POSE}   z = {HOME_POSE[2]*1000:.0f} mm")
    print(f"DROP  {DROP_POSE}   z = {DROP_POSE[2]*1000:.0f} mm")
    print(f"Transit height: {TRANSIT_Z_MM} mm")
    print("\n  1 home    2 drop    3 above home    4 above drop")
    print("  o suction ON    f suction OFF")
    print("  s STOP    q quit")
    print("Hand on the e-stop.\n")

    while True:
        try:
            key = input("> ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nSwitching the suction off.")
            try:
                vacuum_off()
            except OSError:
                print("  Could not switch it off. Use the I/O tab on the pendant.")
            break

        try:
            if key == "1":
                move_to("HOME", pose_str(*HOME_POSE))
            elif key == "2":
                move_to("DROP", pose_str(*DROP_POSE))
            elif key == "3":
                move_to(f"above HOME at {TRANSIT_Z_MM}mm", above(HOME_POSE, TRANSIT_Z_MM))
            elif key == "4":
                move_to(f"above DROP at {TRANSIT_Z_MM}mm", above(DROP_POSE, TRANSIT_Z_MM))
            elif key == "o":
                vacuum_on()
            elif key == "f":
                vacuum_off()
            elif key == "s":
                stop()
            elif key == "q":
                vacuum_off()
                break
            elif key == "":
                continue
            else:
                print("  Use 1, 2, 3, 4, o, f, s or q.")

        except socket.timeout:
            print(f"  No reply from {HOST}. Is Remote Control on?")
        except OSError as exc:
            print(f"  Could not reach the robot: {exc}")


if __name__ == "__main__":
    main()
