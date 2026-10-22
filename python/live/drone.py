"""Run the virtual drone on its own - on this machine, or on a Raspberry Pi.

On the Raspberry Pi (the Pi plays the drone; the laptop runs DroneGuard):

    python -m live.drone --serve --host 192.168.1.5       (the laptop's IP)

    It waits on the ground, sending HEARTBEATs, and flies whatever the
    laptop's live dashboard asks for - then waits again. Ctrl+C to stop.

On one machine, two terminals (no dashboard):

    terminal 1:   python -m live.monitor --connect udpin:127.0.0.1:14550
    terminal 2:   python -m live.drone --speed 10 --faults
"""

from __future__ import annotations

import argparse
import threading
import time

import live  # noqa: F401  (MAVLink 2)

from .autopilot import FlightResult, fly, open_drone_link, recv
from .flights import build_flight
from .mavlink_io import (CMD_START_FLIGHT, MODE_IDLE, heartbeat, parse_start, status_text)
from .virtual_drone import VirtualDrone


def say(conn, text: str) -> None:
    print(time.strftime("%H:%M:%S"), text, flush=True)
    conn.mav.send(status_text(text))


def run_flight(conn, plan, faults, speed, stop_event) -> FlightResult:
    flight = build_flight(plan, faults)
    drone = VirtualDrone(flight.p, flight.c, flight.n_motors)
    say(conn, f"takeoff {speed:g}x " + ("planned drone" if plan else "reference drone"))
    result = fly(conn, drone, flight.c.Cap_Ah, flight.n_motors, speed, stop_event,
                 flight.fault_ramp_s,
                 on_rtl=lambda: say(conn, f"RTL received at {drone.t:.0f} s"))
    soc = drone.telemetry().soc
    if result.how == FlightResult.STOPPED:
        say(conn, f"stopped at {drone.t:.0f} s")
    elif soc <= 0:
        say(conn, f"battery empty {drone.telemetry().dist_home:.0f} m out")
    else:
        say(conn, f"landed, {100 * soc:.0f}% charge left")
    return result


def serve(host: str, port: int, stop_event: threading.Event | None = None,
          bind_host: str = "0.0.0.0") -> None:
    """Wait for flight requests from DroneGuard at (host, port) and fly them."""
    stop_event = stop_event or threading.Event()
    conn = open_drone_link(host, port, bind_host=bind_host)
    print(f"virtual drone ready - talking to DroneGuard at {host}:{port}  (Ctrl+C to stop)",
          flush=True)
    pending = None
    last_hb = 0.0
    try:
        while not stop_event.is_set():
            if pending is None:
                if time.monotonic() - last_hb >= 1.0:          # on the ground, waiting
                    conn.mav.send(heartbeat(MODE_IDLE, False))
                    last_hb = time.monotonic()
                msg = recv(conn, blocking=True, timeout=0.2)
                if msg is not None and msg.get_type() == "COMMAND_LONG" \
                        and msg.command == CMD_START_FLIGHT:
                    pending = msg
                continue
            plan, faults, speed = parse_start(pending)
            pending = None
            try:
                build_flight(plan, faults)                     # can it be built at all?
            except ValueError as e:
                conn.mav.command_ack_send(CMD_START_FLIGHT, 4)   # MAV_RESULT_FAILED
                say(conn, f"cannot fly that: {e}"[:50])
                continue
            conn.mav.command_ack_send(CMD_START_FLIGHT, 0)
            result = run_flight(conn, plan, faults, max(speed, 0.5), stop_event)
            if result.how == FlightResult.RESTART:
                pending = result.restart_msg
    finally:
        conn.close()


def main():
    ap = argparse.ArgumentParser(prog="python -m live.drone")
    ap.add_argument("--serve", action="store_true",
                    help="wait for flights requested by the live dashboard (use on the Pi)")
    ap.add_argument("--host", default="127.0.0.1", help="where DroneGuard runs (the laptop's IP)")
    ap.add_argument("--port", type=int, default=14550, help="DroneGuard's UDP port")
    ap.add_argument("--speed", type=float, default=10.0, help="times real time (without --serve)")
    ap.add_argument("--faults", action="store_true",
                    help="scheduled reference faults (without --serve)")
    a = ap.parse_args()

    try:
        if a.serve:
            serve(a.host, a.port)
        else:
            conn = open_drone_link(a.host, a.port,
                                   bind_host="127.0.0.1" if a.host == "127.0.0.1" else "0.0.0.0")
            run_flight(conn, None, a.faults, a.speed, threading.Event())
            conn.close()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
