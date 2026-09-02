import socket

HOST = "192.168.1.10"   # <-- your robot's IP
PORT = 30002


def send(script):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((HOST, PORT))
    s.sendall(script.encode())
    s.close()


def stop():
    try:
        send("stopj(2.0)\n")
        print("stop sent")
    except Exception as e:
        print(f"could not send stop: {e}")


script = (
    "def test():\n"
    # "  movej(p[-0.150, -0.400, 0.400, 3.14, 0, 0], a=0.1, v=0.05)\n"
    "  movej(p[-0.450, -0.165, 0.150, 2.845, -1.294, 0.051], a=0.1, v=0.05)\n"
    "  popup(\"Move done\", \"Test\", False, False, False)\n"
    "end\n"
)

try:
    send(script)
    print("sent - press Ctrl+C to stop the arm")
    while True:
        pass
except KeyboardInterrupt:
    print("\nstopping")
    stop()