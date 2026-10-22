"""DroneGuard as it runs on the Raspberry Pi: MAVLink in, health + RTH advice out.

    python -m live.monitor                                  listen on UDP 14550 (SITL)
    python -m live.monitor --connect /dev/serial0 --baud 921600   on the Pi
    python -m live.monitor --spec my_drone.json             limits from the planner

It reads telemetry, runs the same DroneGuard module as the simulation every
0.1 s, and when the module decides the drone must turn back early (out of
energy margin, or health ALARM) it sends RETURN_TO_LAUNCH - once. Reaching the
drop point is the autopilot's own business, so no command is sent for that.
"""

from __future__ import annotations

import argparse
import threading
import time

import numpy as np

from droneguard.health import ALARM, WARN
from droneguard.module import DroneGuardModule
from droneguard.sim import SimResult
from droneguard.thresholds import Thresholds

from .mavlink_io import MAV_CMD_NAV_RETURN_TO_LAUNCH, Frame, FrameDecoder, rtl_command

ALERT_NAME = {0: "OK", WARN: "WARN", ALARM: "ALARM"}
ALERT_CLEAR_S = 5.0


class Monitor:
    def __init__(self, c: Thresholds, use_dynamic: float = 1.0, n_motors: int = 4):
        self.c = c
        self.use_dynamic = use_dynamic
        self.decoder = FrameDecoder(c.Cap_Ah)
        self.module = DroneGuardModule(c)
        self.rows: list[tuple] = []      # simData column order (see droneguard.sim.COLUMNS)
        self.events: list[dict] = []
        self.rtl_sent_at = None
        self.no_temperature = False
        self.messages = 0
        self.last_heard = None
        self._alert = 0
        self._state = 0
        self._empty = False
        self._clear_since = None
        self.lock = threading.Lock()

    # ---- input ------------------------------------------------------------
    def on_message(self, msg) -> list:
        """Feed one MAVLink message. Returns messages to send back (maybe RTL)."""
        self.messages += 1
        self.last_heard = time.monotonic()
        if msg.get_type() == "COMMAND_ACK" and msg.command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            t = self.rows[-1][14] if self.rows else 0.0
            if msg.result == 0:
                self._event(t, "ok", "autopilot accepted RETURN TO LAUNCH")
            else:
                self._event(t, "alarm", f"autopilot REFUSED RETURN TO LAUNCH (MAV_RESULT "
                                        f"{msg.result}) - the pilot must bring it home")
            return []
        f = self.decoder.feed(msg)
        return self.tick(f) if f is not None else []

    def tick(self, f: Frame) -> list:
        c = self.c
        if f.T_motor is None and not self.no_temperature:
            self.no_temperature = True
            self._event(f.t, "info", "no ESC temperature in the telemetry - "
                                     "motor health uses current only")
        T = f.T_motor if f.T_motor is not None else c.T_good
        o = self.module.step(T, f.I_motor, f.V_batt, f.soc, f.dist_home, self.use_dynamic)
        if not self.rows:
            self._event(f.t, "info", "telemetry received - DroneGuard is monitoring")
        with self.lock:
            self.rows.append((f.alt, f.speed, f.dist_home, T, f.I_motor, f.V_batt, f.soc,
                              o.motor_health, o.batt_health, o.overall_health, o.alert,
                              o.rth_threshold, o.soc_needed, o.mission_state, f.t))

        # Event log: a worse alert is logged at once; a better one only once it
        # has held for ALERT_CLEAR_S, so a score hovering on a limit doesn't spam.
        # (The module's own alert output is not filtered - only this log is.)
        if o.alert > self._alert:
            self._log_alert(f.t, o)
            self._clear_since = None
        elif o.alert < self._alert:
            if self._clear_since is None:
                self._clear_since = f.t
            elif f.t - self._clear_since >= ALERT_CLEAR_S:
                self._log_alert(f.t, o)
                self._clear_since = None
        else:
            self._clear_since = None

        out = []
        if o.mission_state != self._state:
            if o.mission_state == 1:
                if f.dist_home >= c.Range:
                    self._event(f.t, "info", f"drop point reached at {f.dist_home:.0f} m "
                                             "- autopilot heads home")
                else:
                    why = ("health ALARM - not fit to continue" if o.alert >= ALARM
                           else f"charge is down to {100 * f.soc:.0f}%, what the trip home "
                                f"needs plus the {100 * c.Reserve:.0f}% reserve")
                    self._event(f.t, "alarm", f"RETURN TO LAUNCH sent at {f.dist_home:.0f} m: {why}")
                    self.rtl_sent_at = f.t
                    out.append(rtl_command())
            elif o.mission_state == 2:
                self._event(f.t, "ok", f"home - landed with {100 * f.soc:.0f}% charge")
            self._state = o.mission_state
        if f.soc <= 0 and not self._empty:
            self._empty = True
            self._event(f.t, "alarm", f"BATTERY EMPTY {f.dist_home:.0f} m from home")
        return out

    def _log_alert(self, t: float, o) -> None:
        level = ALERT_NAME[o.alert]
        self._event(t, {"OK": "ok", "WARN": "warn", "ALARM": "alarm"}[level],
                    f"{level}: health {o.overall_health:.0f} "
                    f"(motor {o.motor_health:.0f}, battery {o.batt_health:.0f})")
        self._alert = o.alert

    def _event(self, t: float, level: str, text: str) -> None:
        with self.lock:
            self.events.append({"t": round(t, 1), "level": level, "text": text})

    # ---- output -----------------------------------------------------------
    def result(self) -> SimResult:
        """The monitored flight in the simulation's format, for droneguard.summarize."""
        with self.lock:
            a = np.array(self.rows) if self.rows else np.zeros((0, 15))
        return SimResult(a[:, 14], a[:, :14], None)


def main():
    from pymavlink import mavutil
    from .spec import load_spec

    ap = argparse.ArgumentParser(prog="python -m live.monitor")
    ap.add_argument("--connect", default="udpin:0.0.0.0:14550",
                    help="pymavlink connection string, e.g. udpin:0.0.0.0:14550 or /dev/serial0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--spec", help="JSON spec from the planner (limits + battery)")
    ap.add_argument("--fixed", action="store_true", help="naive fixed 25%% rule")
    a = ap.parse_args()

    c = load_spec(a.spec) if a.spec else Thresholds()
    mon = Monitor(c, use_dynamic=0.0 if a.fixed else 1.0)
    conn = mavutil.mavlink_connection(a.connect, baud=a.baud, source_system=42,
                                      dialect="ardupilotmega")
    print(f"DroneGuard monitor listening on {a.connect}  (Ctrl+C to stop)", flush=True)
    shown_events, last_print = 0, 0.0
    try:
        while True:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            for out in mon.on_message(msg):
                conn.mav.send(out)
            for e in mon.events[shown_events:]:
                print(f"[{e['t']:7.1f} s] {e['text']}", flush=True)
            shown_events = len(mon.events)
            if mon.rows and time.monotonic() - last_print > 1.0:
                r = mon.rows[-1]
                print(f"  t={r[14]:6.1f}s  dist={r[2]:6.0f}m  charge={100 * r[6]:3.0f}%  "
                      f"needed={100 * r[11]:3.0f}%  health={r[9]:3.0f}  {ALERT_NAME[int(r[10])]}",
                      flush=True)
                last_print = time.monotonic()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
