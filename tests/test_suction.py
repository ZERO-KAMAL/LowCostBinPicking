"""Interactive suction test for the UR10 vacuum gripper.

Keeps running and waits for a keypress:

    o  vacuum ON
    f  vacuum OFF (with blow-off puff)
    q  quit (switches the vacuum off first)

Does nothing but switch the outputs - no arm movement, so it is safe to run
with the robot parked anywhere.

Wiring:
    DO0 = vacuum on
    DO1 = blow-off / release puff

Usage:
    python src/test_suction.py
"""

import socket

# Must match the robot's IP in send_pick_position.py
ROBOT_IP = "172.17.128.102"
ROBOT_PORT = 30002

VACUUM_OUT = 0        # DO0 - suction
BLOWOFF_OUT = 1       # DO1 - release puff
BLOWOFF_S = 0.3       # how long to puff air when releasing


def send(script: str) -> None:
    """Send one URScript program to the controller."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    try:
        s.connect((ROBOT_IP, ROBOT_PORT))
        s.sendall(script.encode())
    finally:
        s.close()


def vacuum_on() -> None:
    send(
        "def vac_on():\n"
        f"  set_digital_out({VACUUM_OUT}, True)\n"
        "end\n"
    )
    print(f"  DO{VACUUM_OUT} ON  - suction running")


def vacuum_off() -> None:
    send(
        "def vac_off():\n"
        f"  set_digital_out({VACUUM_OUT}, False)\n"
        f"  set_digital_out({BLOWOFF_OUT}, True)\n"
        f"  sleep({BLOWOFF_S})\n"
        f"  set_digital_out({BLOWOFF_OUT}, False)\n"
        "end\n"
    )
    print(f"  DO{VACUUM_OUT} OFF - DO{BLOWOFF_OUT} puffed to release")


def main() -> None:
    print(f"Robot: {ROBOT_IP}:{ROBOT_PORT}")
    print("Keys:  o = vacuum ON   f = vacuum OFF   q = quit")

    try:
        while True:
            key = input("> ").strip().lower()

            if key == "o":
                vacuum_on()
            elif key == "f":
                vacuum_off()
            elif key == "q":
                vacuum_off()
                print("Bye.")
                break
            elif key == "":
                continue
            else:
                print("  Unknown key. Use o, f or q.")

    except socket.timeout:
        print(f"No reply from {ROBOT_IP}. Check the cable and that the robot is on.")
    except OSError as exc:
        print(f"Could not reach the robot: {exc}")
    except (KeyboardInterrupt, EOFError):
        print("\nInterrupted - switching the vacuum off.")
        try:
            vacuum_off()
        except OSError:
            print("Could not switch it off. Use the I/O tab on the pendant.")


if __name__ == "__main__":
    main()
