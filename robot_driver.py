"""
UR5 driver for automated 2D bin picking with force-guided descent and a
pneumatic (vacuum) gripper.

This replaces send_pick_position.py / send_pick_position3.py.

WHAT IS DIFFERENT FROM YOUR OLD FILE
  1. No fixed PICK_Z_MM. The arm descends under force control and stops when it
     touches the object. That is how a 2D vision system gets its missing Z.
  2. The vacuum is switched by a digital output, and (optionally) verified by a
     vacuum switch on a digital input, so the program knows whether the pick
     actually succeeded.
  3. The whole cycle (approach -> touch -> suck -> lift -> drop -> release ->
     home) runs as ONE URScript program on the robot. The robot reports each
     step back to this PC over a socket, so Python waits for a real "done"
     message instead of guessing with time.sleep().

BEFORE YOU RUN THIS, ON THE PENDANT
  * Installation -> TCP: set the TCP to the tip of the suction cup, and enter
    the correct payload and centre of gravity. Force mode is worthless if the
    payload is wrong - the robot will misread gravity as contact force.
  * Installation -> I/O: make sure the digital output driving the vacuum
    ejector valve is not owned by a URCap.
  * Keep the physical emergency stop within reach. Auto mode moves the arm
    without asking.

TEST ORDER (do not skip)
  1. python robot_driver.py --dry-run      -> prints the URScript, sends nothing
  2. python robot_driver.py --selftest     -> home, drop, home. No descent.
  3. python robot_driver.py --touch-test X Y  -> one force descent, no suction
  4. python robot_driver.py --pick-test X Y   -> one full cycle
"""

from __future__ import annotations

import argparse
import queue
import socket
import threading
import time
from typing import Optional

# ============================================================================
# 1. CONNECTION
# ============================================================================
ROBOT_HOST = "192.168.1.10"   # robot IP (same as your old file)
ROBOT_PORT = 30002            # URScript secondary interface

# The IP of THIS PC as the robot sees it. The robot opens a socket back to
# here to report progress. Find it with `ipconfig` / `ip addr` on the machine
# running the vision code. It must be on the same subnet as the robot.
PC_HOST = "192.168.1.20"      # <-- CHANGE ME
PC_PORT = 40001               # any free port; allow it through the firewall

# ============================================================================
# 2. ROBOT MODEL
# ============================================================================
# True  = UR5e / e-Series (real force-torque sensor in the wrist).
# False = UR5 CB3 (force is estimated from joint currents: much noisier,
#         roughly +/-10 N in Z, so PUSH_FORCE_N has to be higher and a soft
#         bellows suction cup is strongly recommended).
E_SERIES = True

# ============================================================================
# 3. HEIGHTS (mm, robot BASE frame)
# ============================================================================
APPROACH_Z_MM = 150.0    # safe hover height, clear of everything in the bin
SEARCH_START_Z_MM = 80.0  # where force-controlled descent begins.
#                           MUST be above the tallest object in the bin.
Z_FLOOR_MM = 20.0        # hard limit. The TCP is never commanded below this,
#                          even if contact is never detected. Set it just above
#                          the bin floor so a missed detection cannot crash.

# ============================================================================
# 4. FORCE SEARCH
# ============================================================================
PUSH_FORCE_N = 12.0        # downward push while searching. e-Series: 8-15 N.
#                            CB3: try 20-25 N, it cannot resolve less.
DESCEND_SPEED = 0.02       # m/s, max speed of the compliant Z axis
SEARCH_TIMEOUT_S = 6.0     # give up if nothing is touched in this time
STALL_TICKS = 5            # consecutive 50 ms samples with no downward motion
#                            before contact is declared (~0.25 s)
STALL_EPS_M = 0.0002       # movement smaller than this counts as "stopped"

# ============================================================================
# 5. VACUUM GRIPPER I/O
# ============================================================================
SUCTION_DO = 0             # digital output driving the ejector valve
SUCTION_ON_TOOL_IO = False  # True -> tool connector output, False -> control box

BLOWOFF_DO: Optional[int] = None   # optional blow-off output, None to disable
BLOWOFF_ON_TOOL_IO = False
BLOWOFF_PULSE_S = 0.3

VACUUM_DI: Optional[int] = None    # vacuum switch feedback input.
#                                    Set to e.g. 0 if you have one, None if not.
#                                    Without it the robot cannot tell a real
#                                    pick from a miss - worth wiring up.
VACUUM_DI_TOOL = False
VACUUM_ACTIVE_HIGH = True

VACUUM_BUILD_S = 0.4       # time to let the vacuum form before lifting
VACUUM_SETTLE_S = 0.3      # extra dwell after release at the drop point

# ============================================================================
# 6. ORIENTATION AND FIXED POSES (from your working setup)
# ============================================================================
GRIPPER_RX = 2.866
GRIPPER_RY = -1.214
RZ_OFFSET = 0.0            # added to the PCA angle. Irrelevant for a round
#                            suction cup - leave at 0 unless the cup is shaped.

DROP_POSE = [-0.206, -0.306, 0.150, 2.866, -1.214, 0.069]
HOME_POSE = [-0.2058, -0.1362, 0.150, 2.866, -1.214, 0.069]

# ============================================================================
# 7. SPEEDS - keep these low while testing
# ============================================================================
JOINT_ACC = 0.3
JOINT_VEL = 0.1
LINEAR_ACC = 0.3
LINEAR_VEL = 0.05
STOP_DECEL = 2.0

DRY_RUN = False            # True -> print the script, send nothing


# ============================================================================
# URScript helpers
# ============================================================================
def _pose(x, y, z, rx, ry, rz) -> str:
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def _set_do(pin: int, state: bool, tool: bool) -> str:
    fn = "set_tool_digital_out" if tool else "set_digital_out"
    return f"{fn}({pin}, {'True' if state else 'False'})"


def _get_di(pin: int, tool: bool) -> str:
    fn = "get_tool_digital_in" if tool else "get_digital_in"
    return f"{fn}({pin})"


def _send_raw(script: str, description: str) -> bool:
    """Open a socket to the robot, send one URScript block, close."""
    if DRY_RUN:
        print(f"--- DRY RUN: {description} ---")
        print(script)
        print("--- end ---")
        return True
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((ROBOT_HOST, ROBOT_PORT))
        s.sendall(script.encode("utf-8"))
        time.sleep(0.2)
        s.close()
        print(f"{description} sent.")
        return True
    except Exception as exc:
        print(f"ERROR ({description}): {exc}")
        return False


# ============================================================================
# Status server: the robot talks back to us
# ============================================================================
class StatusServer(threading.Thread):
    """Listens for progress messages the URScript program sends to this PC.

    Each line the robot sends becomes an event in a queue. This is what lets
    the vision loop know the arm has genuinely finished, instead of sleeping
    for a fixed number of seconds and hoping.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = PC_PORT):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.events: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._srv: Optional[socket.socket] = None
        self.last_event: Optional[str] = None

    def run(self) -> None:
        try:
            self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._srv.bind((self.host, self.port))
            self._srv.listen(2)
            self._srv.settimeout(1.0)
            print(f"Status server listening on {self.host}:{self.port}")
        except Exception as exc:
            print(f"Could not start status server: {exc}")
            return

        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle(conn)

        try:
            self._srv.close()
        except Exception:
            pass

    def _handle(self, conn: socket.socket) -> None:
        conn.settimeout(1.0)
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = conn.recv(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = line.decode("utf-8", "ignore").strip()
                if msg:
                    self.last_event = msg
                    self.events.put(msg)
                    print(f"  [robot] {msg}")
        try:
            conn.close()
        except Exception:
            pass

    def drain(self) -> None:
        """Throw away stale events before starting a new cycle."""
        while True:
            try:
                self.events.get_nowait()
            except queue.Empty:
                return

    def wait_for(self, wanted: tuple[str, ...], timeout: float) -> Optional[str]:
        """Block until one of `wanted` arrives. Returns None on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if msg in wanted:
                return msg
        return None

    def shutdown(self) -> None:
        self._stop.set()


# ============================================================================
# Simple moves
# ============================================================================
def send_stop() -> None:
    """Decelerate to a halt. A CONVENIENCE, not a safety device."""
    if DRY_RUN:
        print("DRY RUN: stopj")
        return
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((ROBOT_HOST, ROBOT_PORT))
        s.sendall(f"stopj({STOP_DECEL})\n".encode("utf-8"))
        s.close()
        print("Stop command sent.")
    except Exception as exc:
        print(f"Could not send stop: {exc}")
        print("If the arm is still moving, use the physical emergency stop.")


def suction_off() -> None:
    """Kill the vacuum immediately, independently of any running program."""
    script = (
        "def vac_off():\n"
        f"  {_set_do(SUCTION_DO, False, SUCTION_ON_TOOL_IO)}\n"
        "end\n"
    )
    _send_raw(script, "Suction off")


def send_home() -> None:
    script = (
        "def go_home():\n"
        f"  movej({_pose(*HOME_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "end\n"
    )
    print("Moving to home position.")
    _send_raw(script, "Home command")


def send_drop() -> None:
    script = (
        "def go_drop():\n"
        f"  movej({_pose(*DROP_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})\n"
        "end\n"
    )
    print("Moving to drop position.")
    _send_raw(script, "Drop command")


# ============================================================================
# The full pick cycle, as one URScript program
# ============================================================================
def build_cycle_script(x_mm: float, y_mm: float, rz: float = 0.0,
                       do_suction: bool = True, do_drop: bool = True) -> str:
    """Generate the URScript for one complete pick-and-place cycle.

    Sequence:
      approach above (x, y)  ->  rapid down to SEARCH_START_Z
      ->  force-controlled descent until the cup touches the object
      ->  vacuum on, hold pressure, verify
      ->  lift to approach height  ->  move to drop  ->  release
      ->  return home  ->  report CYCLE_OK

    do_suction=False gives a touch-only test (descend, report contact, lift).
    """
    x_m = x_mm / 1000.0
    y_m = y_mm / 1000.0
    rz_cmd = rz + RZ_OFFSET

    p_approach = _pose(x_m, y_m, APPROACH_Z_MM / 1000.0, GRIPPER_RX, GRIPPER_RY, rz_cmd)
    p_search = _pose(x_m, y_m, SEARCH_START_Z_MM / 1000.0, GRIPPER_RX, GRIPPER_RY, rz_cmd)
    z_floor_m = Z_FLOOR_MM / 1000.0
    max_ticks = int(SEARCH_TIMEOUT_S / 0.05)

    L: list[str] = []
    a = L.append

    a("def bin_pick_cycle():")
    a(f'  sock_ok = socket_open("{PC_HOST}", {PC_PORT}, "pc")')
    a('  if (sock_ok):')
    a('    socket_send_line("START", "pc")')
    a('  end')

    # --- approach -------------------------------------------------------
    a(f"  movej({p_approach}, a={JOINT_ACC}, v={JOINT_VEL})")
    a(f"  movel({p_search}, a={LINEAR_ACC}, v={LINEAR_VEL})")
    a('  if (sock_ok):')
    a('    socket_send_line("AT_SEARCH_START", "pc")')
    a('  end')

    # --- force-controlled descent ---------------------------------------
    if E_SERIES:
        a("  zero_ftsensor()")
    a("  force_mode(p[0,0,0,0,0,0], [0,0,1,0,0,0], "
      f"[0,0,-{PUSH_FORCE_N},0,0,0], 2, "
      f"[0.05,0.05,{DESCEND_SPEED},0.10,0.10,0.10])")
    a("  p_prev = get_actual_tcp_pose()")
    a("  z_prev = p_prev[2]")
    a("  stalled = 0")
    a("  ticks = 0")
    a("  touched = False")
    a(f"  while (ticks < {max_ticks}):")
    a("    sleep(0.05)")
    a("    p_now = get_actual_tcp_pose()")
    a("    z_now = p_now[2]")
    a(f"    if (z_prev - z_now < {STALL_EPS_M}):")
    a("      stalled = stalled + 1")
    a("    else:")
    a("      stalled = 0")
    a("    end")
    a("    z_prev = z_now")
    # 10 ticks of grace so the initial acceleration is not read as a stall
    a(f"    if (ticks > 10 and stalled >= {STALL_TICKS}):")
    a("      touched = True")
    a("      break")
    a("    end")
    a(f"    if (z_now <= {z_floor_m:.4f}):")
    a("      touched = True")
    a("      break")
    a("    end")
    a("    ticks = ticks + 1")
    a("  end")
    a('  if (sock_ok):')
    a("    if (touched):")
    a('      socket_send_line("CONTACT", "pc")')
    a("    else:")
    a('      socket_send_line("NO_CONTACT", "pc")')
    a("    end")
    a("  end")

    grip_ok = "True"
    if do_suction:
        # Vacuum comes on while force mode still holds the cup against the part.
        a(f"  {_set_do(SUCTION_DO, True, SUCTION_ON_TOOL_IO)}")
        a(f"  sleep({VACUUM_BUILD_S})")
        if VACUUM_DI is not None:
            cmp_val = "True" if VACUUM_ACTIVE_HIGH else "False"
            a(f"  grip = ({_get_di(VACUUM_DI, VACUUM_DI_TOOL)} == {cmp_val})")
            grip_ok = "grip"
        else:
            a("  grip = True")
            grip_ok = "grip"

    a("  end_force_mode()")
    a(f"  movel({p_approach}, a={LINEAR_ACC}, v={LINEAR_VEL})")

    if do_suction:
        a(f"  if (not {grip_ok}):")
        a(f"    {_set_do(SUCTION_DO, False, SUCTION_ON_TOOL_IO)}")
        a(f"    movej({_pose(*HOME_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})")
        a('    if (sock_ok):')
        a('      socket_send_line("VACUUM_FAIL", "pc")')
        a('      socket_send_line("CYCLE_FAIL", "pc")')
        a('      socket_close("pc")')
        a("    end")
        a("    halt")
        a("  end")
        a('  if (sock_ok):')
        a('    socket_send_line("VACUUM_OK", "pc")')
        a("  end")

    # --- drop -----------------------------------------------------------
    if do_drop and do_suction:
        a(f"  movej({_pose(*DROP_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})")
        a(f"  {_set_do(SUCTION_DO, False, SUCTION_ON_TOOL_IO)}")
        if BLOWOFF_DO is not None:
            a(f"  {_set_do(BLOWOFF_DO, True, BLOWOFF_ON_TOOL_IO)}")
            a(f"  sleep({BLOWOFF_PULSE_S})")
            a(f"  {_set_do(BLOWOFF_DO, False, BLOWOFF_ON_TOOL_IO)}")
        a(f"  sleep({VACUUM_SETTLE_S})")
        a('  if (sock_ok):')
        a('    socket_send_line("DROPPED", "pc")')
        a("  end")

    # --- home -----------------------------------------------------------
    a(f"  movej({_pose(*HOME_POSE)}, a={JOINT_ACC}, v={JOINT_VEL})")
    a('  if (sock_ok):')
    a('    socket_send_line("AT_HOME", "pc")')
    a('    socket_send_line("CYCLE_OK", "pc")')
    a('    socket_close("pc")')
    a("  end")
    a("end")
    a("")
    return "\n".join(L)


def run_pick_cycle(x_mm: float, y_mm: float, rz: float = 0.0,
                   status: Optional[StatusServer] = None,
                   timeout_s: float = 60.0,
                   do_suction: bool = True, do_drop: bool = True) -> str:
    """Send one cycle and wait for the robot to report back.

    Returns one of: CYCLE_OK, CYCLE_FAIL, SEND_FAILED, TIMEOUT.
    """
    script = build_cycle_script(x_mm, y_mm, rz, do_suction=do_suction, do_drop=do_drop)

    if status is not None:
        status.drain()

    print(f"PICK cycle -> X={x_mm:.1f}mm Y={y_mm:.1f}mm rz={rz + RZ_OFFSET:.3f}rad")
    if not _send_raw(script, "Pick cycle"):
        return "SEND_FAILED"

    if DRY_RUN:
        return "CYCLE_OK"

    if status is None:
        # No handshake available: fall back to a fixed wait.
        time.sleep(min(timeout_s, 20.0))
        return "CYCLE_OK"

    result = status.wait_for(("CYCLE_OK", "CYCLE_FAIL"), timeout_s)
    if result is None:
        print("TIMEOUT: no completion message from the robot.")
        print("Check PC_HOST, the firewall, and whether the arm is still moving.")
        return "TIMEOUT"
    return result


# ============================================================================
# Standalone tests
# ============================================================================
def main() -> None:
    global DRY_RUN

    parser = argparse.ArgumentParser(description="UR5 vacuum pick driver tests.")
    parser.add_argument("--dry-run", action="store_true", help="Print URScript, send nothing.")
    parser.add_argument("--selftest", action="store_true", help="home -> drop -> home only.")
    parser.add_argument("--touch-test", nargs=2, type=float, metavar=("X_MM", "Y_MM"),
                        help="One force descent at X Y. No suction, no drop.")
    parser.add_argument("--pick-test", nargs=2, type=float, metavar=("X_MM", "Y_MM"),
                        help="One full pick cycle at X Y.")
    args = parser.parse_args()

    DRY_RUN = args.dry_run

    status = None
    if not DRY_RUN:
        status = StatusServer()
        status.start()
        time.sleep(0.5)

    try:
        if args.selftest:
            send_home()
            time.sleep(6)
            send_drop()
            time.sleep(6)
            send_home()
        elif args.touch_test:
            x, y = args.touch_test
            print(run_pick_cycle(x, y, status=status, do_suction=False, do_drop=False))
        elif args.pick_test:
            x, y = args.pick_test
            print(run_pick_cycle(x, y, status=status))
        else:
            # Default: show the generated script for inspection.
            print(build_cycle_script(-450.0, -165.0, 0.069))
    except KeyboardInterrupt:
        print("\nInterrupted - stopping robot and killing vacuum.")
        send_stop()
        suction_off()
    finally:
        if status is not None:
            status.shutdown()


if __name__ == "__main__":
    main()
