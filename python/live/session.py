"""Connect a virtual drone to the DroneGuard monitor.

    run_lockstep(...)   one loop, MAVLink bytes passed in memory - deterministic,
                        same tick order as the simulation, used by the tests
    LiveSession(...)    two threads talking real MAVLink over UDP on localhost,
                        paced against the wall clock - what the dashboard runs
"""

import socket
import threading
import time

from droneguard.params import SimParams
from droneguard.thresholds import Thresholds

from .mavlink_io import DRONE_SYSTEM, DRONEGUARD_SYSTEM, encode_frame, link
from .monitor import Monitor
from .virtual_drone import VirtualDrone


def run_lockstep(p: SimParams, c: Thresholds, use_dynamic: float = 1.0,
                 n_motors: int = 4) -> tuple[Monitor, VirtualDrone]:
    drone, mon = VirtualDrone(p, c, n_motors), Monitor(c, use_dynamic, n_motors)
    to_ground, to_drone = link(DRONE_SYSTEM), link(DRONEGUARD_SYSTEM)
    while True:
        raw = b"".join(m.pack(to_ground) for m in encode_frame(drone.telemetry(), c.Cap_Ah, n_motors))
        replies = []
        for msg in to_drone.parse_buffer(raw) or []:
            replies += mon.on_message(msg)
        raw = b"".join(m.pack(to_drone) for m in replies)
        for cmd in to_ground.parse_buffer(raw) or []:
            drone.command(cmd)
        if drone.done:
            break
        drone.advance()
    return mon, drone


def _quiet_udp(conn) -> None:
    """Windows reports 'connection reset' on a UDP socket when the other side has
    gone away (WinError 10054). For telemetry that is meaningless - switch it off."""
    sock = getattr(conn, "port", None)
    if sock is not None and hasattr(socket, "SIO_UDP_CONNRESET"):
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass


def _recv(conn, **kw):
    try:
        return conn.recv_match(**kw)
    except ConnectionResetError:
        return None


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class LiveSession:
    """A virtual drone and a monitor in two threads, joined by UDP MAVLink."""

    def __init__(self, p: SimParams, c: Thresholds, speed: float = 10.0,
                 use_dynamic: float = 1.0, n_motors: int = 4, label: str = "",
                 fault_ramp_s: float = 200.0, monitor_port: int | None = None):
        """monitor_port=None runs our own monitor on a free port; a number means
        "fly only, and send to whatever monitor is listening there"."""
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.p, self.c, self.speed, self.label = p, c, speed, label
        self.fault_ramp_s = fault_ramp_s
        self.n_motors = n_motors
        self.drone = VirtualDrone(p, c, n_motors)
        self.monitor = Monitor(c, use_dynamic, n_motors)
        self.own_monitor = monitor_port is None
        self.port = free_udp_port() if self.own_monitor else monitor_port
        self.stop_event = threading.Event()
        self.finished = threading.Event()
        self.commands_received = 0
        self._threads = []
        self._pending_faults = []
        self._lock = threading.Lock()

    def start(self) -> None:
        self._threads = [threading.Thread(target=self._drone_loop, daemon=True)]
        if self.own_monitor:
            ground = self.mavutil.mavlink_connection(
                f"udpin:127.0.0.1:{self.port}", source_system=DRONEGUARD_SYSTEM,
                dialect="ardupilotmega")
            self._threads.insert(0, threading.Thread(target=self._monitor_loop,
                                                     args=(ground,), daemon=True))
        for th in self._threads:
            th.start()

    def stop(self) -> None:
        self.stop_event.set()
        for th in self._threads:
            th.join(timeout=2)

    def inject(self, kind: str) -> None:
        with self._lock:                 # applied by the drone thread between ticks
            self._pending_faults.append(kind)

    @property
    def running(self) -> bool:
        return bool(self._threads) and not self.finished.is_set()

    # ---- the two sides --------------------------------------------------
    def _drone_loop(self) -> None:
        # Not "udpout": on Windows pymavlink binds udpout to the SAME port it
        # sends to, so the monitor's replies loop back to the monitor. Instead
        # the drone listens on its own port and is told where the monitor is.
        mav = self.mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{free_udp_port()}", source_system=DRONE_SYSTEM,
            dialect="ardupilotmega")
        _quiet_udp(mav)
        ground = ("127.0.0.1", self.port)
        mav.clients.add(ground)
        mav.clients_last_alive[ground] = time.time()
        mav.mav.srcComponent = 1
        d, h = self.drone, self.p.Ts
        t0 = time.perf_counter()
        last_hb = -1.0
        try:
            while not self.stop_event.is_set():
                with self._lock:
                    faults, self._pending_faults = self._pending_faults, []
                for kind in faults:
                    d.inject(kind, self.fault_ramp_s)
                if d.t - last_hb >= 1.0:
                    mav.mav.heartbeat_send(2, 3, 0, 0, 4)   # quadrotor, ArduPilot, active
                    last_hb = d.t
                for m in encode_frame(d.telemetry(), self.c.Cap_Ah, self.n_motors):
                    mav.mav.send(m)
                while (cmd := _recv(mav, blocking=False)) is not None:
                    if d.command(cmd):
                        self.commands_received += 1
                        mav.mav.command_ack_send(cmd.command, 0)
                if d.done:
                    break
                d.advance()
                wait = t0 + d.t / self.speed - time.perf_counter()
                if wait > 0:
                    time.sleep(wait)
        finally:
            time.sleep(0.3)                 # let the last frames land
            self.finished.set()
            mav.close()

    def _monitor_loop(self, ground) -> None:
        _quiet_udp(ground)
        try:
            while not self.stop_event.is_set():
                msg = _recv(ground, blocking=True, timeout=0.2)
                if msg is None:
                    if self.finished.is_set():
                        break
                    continue
                for out in self.monitor.on_message(msg):
                    ground.mav.send(out)
        finally:
            ground.close()
