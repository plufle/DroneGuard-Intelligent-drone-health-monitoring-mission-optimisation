"""DroneGuard laptop dashboard (standalone Python, no MATLAB required)."""
from __future__ import annotations

import json
import sys
import threading
import time
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


@dataclass
class Telemetry:
    time_s: float = 0.0
    distance_m: float = 0.0
    speed_mps: float = 0.0
    motor_temp_c: float = 30.0
    current_a: float = 18.0
    voltage_v: float = 16.44
    soc: float = 1.0
    motor_health: float = 100.0
    battery_health: float = 100.0
    overall_health: float = 100.0
    alert: str = "OK"
    rth_threshold: float = 0.10
    motor_fault_pct: float = 0.0
    battery_fault_pct: float = 0.0
    state: str = "OUTBOUND"
    reason: str = "Following mission"


@dataclass
class SimulationConfig:
    """Mission and aircraft controls exposed by the laptop dashboard."""
    mission_distance_m: float = 1000.0
    payload_kg: float = 4.0
    max_time_s: float = 800.0
    cruise_speed_mps: float = 12.0
    reserve_percent: float = 10.0
    battery_capacity_ah: float = 5.0
    motor_fault_start_s: float = 40.0
    battery_fault_start_s: float = 60.0
    waypoint_action: str = "Stop at waypoint"


class SimulatedDrone:
    """Replace this class later with a MAVLink telemetry adapter."""
    margin = 1.25

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config or SimulationConfig()
        self.t = Telemetry()
        self.t.current_a = 18.0 + 2.4 * self.config.payload_kg
        self._last_current = self.t.current_a
        self._trend = 0.0

    @staticmethod
    def score(value: float, good: float, bad: float) -> float:
        return max(0.0, min(100.0, 100 * (bad - value) / (bad - good)))

    def step(self, dt: float = 1.0) -> Telemetry:
        t = self.t
        t.time_s += dt
        c = self.config
        # Simulated gradual motor wear and battery ageing.
        # Faults begin ramping as soon as their configured start time arrives.
        # The fixed 120-second ramp keeps the dashboard simple and visible.
        motor_fault = max(0.0, min(1.0, (t.time_s - c.motor_fault_start_s) / 120.0))
        batt_fault = max(0.0, min(1.0, (t.time_s - c.battery_fault_start_s) / 120.0))
        t.motor_fault_pct = motor_fault * 100.0
        t.battery_fault_pct = batt_fault * 100.0

        if t.state == "OUTBOUND":
            t.speed_mps = c.cruise_speed_mps
            t.distance_m = min(c.mission_distance_m, t.distance_m + c.cruise_speed_mps * dt)
        elif t.state == "RETURNING":
            t.speed_mps = -c.cruise_speed_mps
            t.distance_m = max(0.0, t.distance_m - c.cruise_speed_mps * dt)
        else:
            t.speed_mps = 0.0

        flying = t.state in ("OUTBOUND", "RETURNING")
        # A payload needs extra lift; 2.4 A/kg is a simple modelled penalty.
        t.current_a = (18.0 + 2.4 * c.payload_kg + 13.0 * motor_fault + 0.15 * abs(t.speed_mps)) if flying else 0.0
        t.motor_temp_c += ((30 + 0.055 * t.current_a**2) - t.motor_temp_c) * dt / 40.0
        resistance = 0.020 * (1 + 3.0 * batt_fault)
        t.soc = max(0.0, t.soc - (t.current_a / 3600 / c.battery_capacity_ah) * (1 + 0.67 * batt_fault) * dt)
        t.voltage_v = 14.0 + 2.8 * t.soc - t.current_a * resistance

        motor = min(self.score(t.motor_temp_c, 60, 100), self.score(t.current_a, 28, 40))
        expected_voltage = 14.0 + 2.8 * t.soc - t.current_a * 0.020
        battery = self.score(expected_voltage - t.voltage_v, 0.10, 1.50)
        t.motor_health, t.battery_health = motor, battery
        t.overall_health = min(motor, battery)
        t.alert = "ALARM" if t.overall_health < 40 else "WARN" if t.overall_health < 70 else "OK"

        raw_trend = (t.current_a - self._last_current) / dt
        self._trend += (raw_trend - self._trend) * dt / 20.0
        self._last_current = t.current_a
        time_home = t.distance_m / c.cruise_speed_mps
        predicted_current = t.current_a + 0.5 * max(0.0, self._trend) * time_home
        needed = predicted_current * time_home / 3600 / c.battery_capacity_ah
        t.rth_threshold = min(1.0, needed * self.margin + c.reserve_percent / 100.0)

        reached_waypoint = t.distance_m >= c.mission_distance_m
        if t.state not in ("HOME", "BATTERY DEAD", "TIME LIMIT", "WAYPOINT REACHED") and t.time_s >= c.max_time_s:
            t.state, t.reason = "TIME LIMIT", "Configured maximum mission time reached"
        elif t.state == "OUTBOUND" and reached_waypoint and c.waypoint_action == "Stop at waypoint":
            t.state, t.reason = "WAYPOINT REACHED", "Configured mission distance achieved — simulation stopped"
        elif t.state == "OUTBOUND" and (reached_waypoint or t.soc <= t.rth_threshold or t.alert == "ALARM"):
            t.state = "RETURNING"
            if t.alert == "ALARM":
                t.reason = "Health ALARM: returning now"
            elif reached_waypoint:
                t.reason = "Mission distance achieved: returning home"
            else:
                t.reason = "Energy margin reached: returning home"
        elif t.state == "RETURNING" and t.distance_m <= 0.0:
            t.state, t.reason = "HOME", "Mission complete: drone is home"
        elif t.state != "HOME" and t.soc <= 0.0:
            t.state, t.reason = "BATTERY DEAD", "Simulated battery depleted"
        return t


drone = SimulatedDrone()
lock = threading.Lock()
history: list[dict] = []
events: list[dict] = [{"time_s": 0, "message": "System ready — simulation is paused."}]
running = False
previous_state = drone.t.state
previous_alert = drone.t.alert


def add_event(message: str) -> None:
    events.append({"time_s": round(drone.t.time_s), "message": message})
    del events[:-12]


def simulator_loop() -> None:
    global previous_alert, previous_state
    while True:
        with lock:
            if running:
                row = asdict(drone.step())
                history.append(row)
                del history[:-240]
                if drone.t.alert != previous_alert:
                    add_event(f"Health alert changed: {previous_alert} → {drone.t.alert}.")
                    previous_alert = drone.t.alert
                if drone.t.state != previous_state:
                    add_event(f"Mission state changed: {previous_state} → {drone.t.state}. {drone.t.reason}")
                    previous_state = drone.t.state
        time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/api/status":
            with lock:
                payload = {"telemetry": asdict(drone.t), "history": history,
                           "events": events, "running": running}
            body = json.dumps(payload).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
            return
        if self.path in ("/", "/index.html"):
            body = (Path(__file__).parent / "index.html").read_bytes()
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.end_headers(); self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self) -> None:
        global running, drone, previous_state, previous_alert
        if self.path == "/api/start":
            with lock:
                if not running:
                    running = True
                    add_event("Simulation started. Drone is departing home.")
            self.send_response(204); self.end_headers()
            return
        if self.path == "/api/reset":
            with lock:
                running = False
                drone = SimulatedDrone()
                history.clear()
                events.clear()
                events.append({"time_s": 0, "message": "Simulation reset — ready to start."})
                previous_state, previous_alert = drone.t.state, drone.t.alert
            self.send_response(204); self.end_headers()
            return
        self.send_error(404)

    def log_message(self, *_: object) -> None:
        pass


if __name__ == "__main__":
    threading.Thread(target=simulator_loop, daemon=True).start()
    # Some Windows installations reserve common development ports such as
    # 8080. Accept an optional port and otherwise try a small safe range.
    requested_port = int(sys.argv[1]) if len(sys.argv) > 1 else None
    ports = [requested_port] if requested_port else [8080, 8081, 8082, 8090]
    server = None
    for port in ports:
        try:
            server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError as error:
            print(f"Cannot use port {port}: {error}")
    if server is None:
        raise SystemExit("No dashboard port was available. Try: python app.py 9000")
    print(f"DroneGuard dashboard: http://localhost:{server.server_port}")
    server.serve_forever()
