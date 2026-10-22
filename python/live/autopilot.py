"""The drone's side of the MAVLink conversation - what a flight controller does.

Used in two places with exactly the same code:
  - the laptop-only live demo (drone and DroneGuard on one machine), and
  - the Raspberry Pi playing the drone (python -m live.drone --serve).

Every tick it sends telemetry; once a second (of flight time) a HEARTBEAT with
the ArduCopter flight mode (AUTO -> RTL -> LAND); twice a second the injected
fault levels (simulation-only ground truth). It obeys RETURN_TO_LAUNCH, and -
simulation only - fault injection, stop, and a new-flight request.
"""

from __future__ import annotations

import socket
import threading
import time

from .mavlink_io import (CMD_INJECT_FAULT, CMD_START_FLIGHT, CMD_STOP_FLIGHT, DRONE_SYSTEM,
                         FAULT_CODES, MODE_AUTO, MODE_LAND, MODE_RTL, encode_frame, heartbeat,
                         truth_messages)
from .virtual_drone import LANDED, OUTBOUND, RETURNING, VirtualDrone

MODE_OF_PHASE = {OUTBOUND: MODE_AUTO, RETURNING: MODE_RTL, LANDED: MODE_LAND}
FAULT_OF_CODE = {v: k for k, v in FAULT_CODES.items()}


def quiet_udp(conn) -> None:
    """Windows reports 'connection reset' on a UDP socket when the other side has
    gone away (WinError 10054). For telemetry that is meaningless - switch it off."""
    sock = getattr(conn, "port", None)
    if sock is not None and hasattr(socket, "SIO_UDP_CONNRESET"):
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass


def recv(conn, **kw):
    try:
        return conn.recv_match(**kw)
    except ConnectionResetError:
        return None


def open_drone_link(target_host: str, target_port: int, bind_host: str = "127.0.0.1",
                    local_port: int = 0):
    """A MAVLink socket for the drone side that sends to (target_host, target_port).

    Not "udpout": on Windows pymavlink binds udpout to the SAME port it sends to,
    so replies loop back to the sender. Instead the drone listens on its own
    port and is told where DroneGuard is; replies come back to that port."""
    import live  # noqa: F401  (MAVLink 2 before pymavlink loads)
    from pymavlink import mavutil
    if local_port == 0:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((bind_host, 0))
            local_port = s.getsockname()[1]
    conn = mavutil.mavlink_connection(f"udpin:{bind_host}:{local_port}",
                                      source_system=DRONE_SYSTEM, dialect="ardupilotmega")
    quiet_udp(conn)
    target = (socket.gethostbyname(target_host), target_port)
    conn.clients.add(target)
    conn.clients_last_alive[target] = time.time()
    conn.mav.srcComponent = 1
    return conn


class FlightResult:
    DONE, STOPPED, RESTART = "done", "stopped", "restart"

    def __init__(self, how: str, restart_msg=None):
        self.how, self.restart_msg = how, restart_msg


def fly(conn, drone: VirtualDrone, cap_Ah: float, n_motors: int, speed: float,
        stop_event: threading.Event, fault_ramp_s: float,
        pending_faults=lambda: [], on_rtl=lambda: None) -> FlightResult:
    """Fly one mission to the end, paced at `speed` x real time."""
    t0 = time.perf_counter()
    last_hb = last_truth = -1.0
    result = FlightResult(FlightResult.DONE)
    while not stop_event.is_set():
        for kind in pending_faults():
            drone.inject(kind, fault_ramp_s)
        if drone.t - last_hb >= 1.0:
            conn.mav.send(heartbeat(MODE_OF_PHASE[drone.phase], not drone.done, n_motors))
            last_hb = drone.t
        if drone.t - last_truth >= 0.5:
            for m in truth_messages(drone.t, drone.fault_levels()):
                conn.mav.send(m)
            last_truth = drone.t
        for m in encode_frame(drone.telemetry(), cap_Ah, n_motors):
            conn.mav.send(m)

        while (cmd := recv(conn, blocking=False)) is not None:
            if cmd.get_type() != "COMMAND_LONG":
                continue
            if drone.command(cmd):                                 # RETURN_TO_LAUNCH
                on_rtl()
                conn.mav.command_ack_send(cmd.command, 0)
            elif cmd.command == CMD_INJECT_FAULT and int(cmd.param1) in FAULT_OF_CODE:
                drone.inject(FAULT_OF_CODE[int(cmd.param1)], fault_ramp_s)
                conn.mav.command_ack_send(cmd.command, 0)
            elif cmd.command == CMD_STOP_FLIGHT:
                conn.mav.command_ack_send(cmd.command, 0)
                return FlightResult(FlightResult.STOPPED)
            elif cmd.command == CMD_START_FLIGHT:
                return FlightResult(FlightResult.RESTART, cmd)

        if drone.done:
            break
        drone.advance()
        wait = t0 + drone.t / speed - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
    else:
        result = FlightResult(FlightResult.STOPPED)
    # Landed (or out of battery): say so a few times so the ground hears it.
    for _ in range(3):
        conn.mav.send(heartbeat(MODE_LAND, False, n_motors))
        for m in encode_frame(drone.telemetry(), cap_Ah, n_motors):
            conn.mav.send(m)
        time.sleep(0.1)
    return result
