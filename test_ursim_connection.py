"""
Simple test script to confirm socket connection to URSim (Docker, running on Mac).

Before running this:
1. Make sure Docker Desktop is open
2. Make sure your URSim container is running (the docker run command from before)
3. Make sure the robot is powered ON and brakes RELEASED on the pendant screen
   (http://127.0.0.1:6080/vnc.html?host=127.0.0.1&port=6080)

This script sends a tiny, safe command using URScript, over the same port (30002)
that a real UR robot uses.
"""

import socket
import time

HOST = "127.0.0.1"   # localhost, since URSim is running on your own Mac via Docker
PORT = 30002          # standard UR "secondary" script interface port

def test_connection():
    print(f"Connecting to URSim at {HOST}:{PORT} ...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((HOST, PORT))
        print("Connected successfully!")

        # Safe test command: just moves to the robot's current position (no real motion)
        # and shows a popup on the pendant to confirm the command was received.
        command = (
            "def test_prog():\n"
            "  movej(get_actual_joint_positions(), a=1.0, v=0.1)\n"
            "  popup(\"Connection test successful!\", \"Test\", False, False, False)\n"
            "end\n"
        )

        s.sendall(command.encode('utf-8'))
        print("Command sent. Check the pendant screen in your browser —")
        print("you should see a popup message confirming success.")

        time.sleep(1)
        s.close()
        print("Connection closed cleanly.")

    except socket.timeout:
        print("ERROR: Connection timed out. Is the URSim container running?")
    except ConnectionRefusedError:
        print("ERROR: Connection refused. Check that:")
        print("  - Docker container is running")
        print("  - Robot is powered on in the pendant screen")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    test_connection()
