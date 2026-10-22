"""Run the virtual drone on its own, as a stand-in for SITL or a real Pixhawk.

    terminal 1:   python -m live.monitor                  (DroneGuard, listens on 14550)
    terminal 2:   python -m live.drone --speed 10         (the drone, sends to 14550)

Exactly the arrangement on the real aircraft: two programs, MAVLink between.
"""

import argparse
from dataclasses import replace

import live  # noqa: F401  (MAVLink 2)
from droneguard.params import SimParams
from droneguard.thresholds import Thresholds

from .session import LiveSession
from .virtual_drone import PHASES


def main():
    ap = argparse.ArgumentParser(prog="python -m live.drone")
    ap.add_argument("--port", type=int, default=14550, help="where DroneGuard listens")
    ap.add_argument("--speed", type=float, default=10.0, help="times real time")
    ap.add_argument("--faults", action="store_true",
                    help="scheduled reference faults (motor from 150 s, battery from 400 s)")
    a = ap.parse_args()

    p = SimParams() if a.faults else replace(SimParams(), MotorFault_Mag=0.0, BattFault_Mag=0.0)
    s = LiveSession(p, Thresholds(), speed=a.speed, monitor_port=a.port)
    print(f"virtual drone sending MAVLink to 127.0.0.1:{a.port} at {a.speed:g}x  (Ctrl+C to stop)")
    s.start()
    last_phase = None
    try:
        while not s.finished.wait(1.0):
            d = s.drone
            if d.phase_name != last_phase:
                print(f"[{d.t:7.1f} s] {d.phase_name}" +
                      (f" (RETURN_TO_LAUNCH received at {d.rtl_at:.1f} s)" if d.rtl_at else ""))
                last_phase = d.phase_name
        d = s.drone
        print(f"[{d.t:7.1f} s] {d.phase_name}, {100 * d.telemetry().soc:.0f}% charge left")
    except KeyboardInterrupt:
        s.stop()


if __name__ == "__main__":
    main()
