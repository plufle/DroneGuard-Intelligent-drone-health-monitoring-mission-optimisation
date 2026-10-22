"""Live monitoring (P4): a virtual drone and DroneGuard joined by real MAVLink.

    .venv/Scripts/python -m unittest -v
"""

import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path

from droneguard import SimParams, Thresholds, summarize
from droneguard.summary import ABORT_ENERGY, ABORT_HEALTH, COMPLETE

try:
    import live  # noqa: F401
    from live.mavlink_io import DRONE_SYSTEM, DRONEGUARD_SYSTEM, Frame, encode_frame, link
    from live.monitor import Monitor
    from live.session import LiveSession, run_lockstep
    from live.spec import load_spec, save_spec
except ImportError:
    live = None

SEVERE = dict(MotorFault_Start=30, MotorFault_Slope=1 / 200, k_eff_fault=0.80)


@unittest.skipIf(live is None, "pymavlink not installed (pip install -r requirements.txt)")
class ThroughMAVLink(unittest.TestCase):
    """The reference results, with every number squeezed through MAVLink fields
    (temperature in whole degC, current in cA, charge used in mAh). They must
    agree with Simulink to within that rounding."""

    def lockstep(self, **changes):
        p = replace(SimParams(), **changes)
        mon, drone = run_lockstep(p, Thresholds(), use_dynamic=p.Use_Dynamic_RTH)
        return summarize(mon.result()), mon, drone

    def test_default_energy_abort(self):
        s, mon, drone = self.lockstep()
        self.assertEqual(s.turn_reason, ABORT_ENERGY)
        self.assertAlmostEqual(s.turn_t, 321, delta=1)
        self.assertAlmostEqual(s.turn_dist, 3817, delta=10)
        self.assertAlmostEqual(s.home_t, 649, delta=1)
        self.assertEqual(round(100 * s.home_soc), 10)
        self.assertAlmostEqual(s.warn_t, 529, delta=2)
        self.assertAlmostEqual(s.alarm_t, 613, delta=2)
        self.assertIsNotNone(drone.rtl_at, "the autopilot must have been told to come home")

    def test_fixed_rule_dies(self):
        s, mon, drone = self.lockstep(Use_Dynamic_RTH=0)
        self.assertEqual(s.turn_reason, COMPLETE)
        self.assertAlmostEqual(s.dead_t, 687, delta=1)
        self.assertAlmostEqual(s.dead_dist, 3913, delta=10)
        self.assertIsNone(drone.rtl_at, "reaching the drop point needs no command")

    def test_severe_motor_fault_health_abort(self):
        s, mon, drone = self.lockstep(**SEVERE)
        self.assertEqual(s.turn_reason, ABORT_HEALTH)
        self.assertAlmostEqual(s.turn_t, 145, delta=1)
        self.assertAlmostEqual(s.turn_dist, 1710, delta=10)
        self.assertAlmostEqual(s.home_t, 298, delta=1)
        rtl = [e for e in mon.events if "RETURN TO LAUNCH" in e["text"]]
        self.assertEqual(len(rtl), 1, "RTL is sent once")
        self.assertIn("health ALARM", rtl[0]["text"])

    def test_without_esc_telemetry(self):
        # PX4/ArduPilot SITL sends no ESC temperature: DroneGuard must carry on,
        # using battery current, and say what it is missing.
        mon = Monitor(Thresholds())
        tx, rx = link(DRONE_SYSTEM), link(DRONEGUARD_SYSTEM)
        f = Frame(t=0.0, T_motor=None, I_motor=18.0, V_batt=16.4, soc=1.0,
                  dist_home=0.0, alt=50.0, speed=0.0)
        raw = b"".join(m.pack(tx) for m in encode_frame(f, 5.0))
        for msg in rx.parse_buffer(raw):
            mon.on_message(msg)
        self.assertTrue(mon.no_temperature)
        self.assertAlmostEqual(mon.rows[-1][4], 18.0)
        self.assertTrue(any("no ESC temperature" in e["text"] for e in mon.events))

    def test_spec_file_round_trip(self):
        c = replace(Thresholds(), Cap_Ah=7.5, I_bad=55.0, Range=3000)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "spec.json"
            save_spec(path, c, note="test")
            self.assertEqual(load_spec(path), c)


@unittest.skipIf(live is None, "pymavlink not installed")
class OverUDP(unittest.TestCase):

    def test_two_programs_over_udp(self):
        s = LiveSession(SimParams(), Thresholds(), speed=200)
        s.start()
        self.assertTrue(s.finished.wait(30), "flight did not finish")
        s.stop()
        r = summarize(s.monitor.result())
        self.assertEqual(s.commands_received, 1)
        self.assertTrue(any("accepted RETURN TO LAUNCH" in e["text"] for e in s.monitor.events))
        self.assertEqual(r.turn_reason, ABORT_ENERGY)
        self.assertAlmostEqual(r.turn_t, 321, delta=2)
        self.assertIsNotNone(r.home_t)
        self.assertGreaterEqual(r.home_soc, 0.09)

    def test_injected_fault_is_caught(self):
        # Clean flight, motor wear injected at the start: health must fall and
        # DroneGuard must bring it home early.
        p = replace(SimParams(), MotorFault_Mag=0.0, BattFault_Mag=0.0)
        s = LiveSession(p, Thresholds(), speed=200, fault_ramp_s=120)
        s.inject("motor")
        s.start()
        self.assertTrue(s.finished.wait(30))
        s.stop()
        r = summarize(s.monitor.result())
        self.assertIsNotNone(r.alarm_t)
        self.assertEqual(r.turn_reason, ABORT_HEALTH)
        self.assertIsNotNone(r.home_t)


@unittest.skipIf(live is None, "pymavlink not installed")
class PiPlaysTheDrone(unittest.TestCase):
    """The Raspberry Pi set-up, both halves on this machine: the drone server
    (what the Pi runs) in a thread, DroneGuard's PiLink listening for it."""

    @classmethod
    def setUpClass(cls):
        import threading
        from live.drone import serve
        from live.remote import PiLink
        from live.session import free_udp_port
        cls.port = free_udp_port()
        cls.link = PiLink(cls.port, bind_host="127.0.0.1")
        cls.stop = threading.Event()
        cls.server = threading.Thread(target=serve, args=("127.0.0.1", cls.port, cls.stop),
                                      kwargs={"bind_host": "127.0.0.1"}, daemon=True)
        cls.server.start()
        deadline = time.time() + 5
        while not cls.link.connected and time.time() < deadline:
            time.sleep(0.05)

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.server.join(timeout=3)
        cls.link.close()

    def fly(self, faults, speed=200, inject=None):
        from live.flights import build_flight
        self.assertTrue(self.link.connected, "the drone server's heartbeat was not heard")
        self.link.start(build_flight(None, faults), None, faults, speed)
        if inject:
            time.sleep(0.3)
            self.link.inject(inject)
        deadline = time.time() + 30
        time.sleep(1.0)
        while self.link.running and time.time() < deadline:
            time.sleep(0.1)
        return summarize(self.link.monitor.result())

    def test_reference_mission_over_the_link(self):
        r = self.fly(faults=True)
        self.assertEqual(r.turn_reason, ABORT_ENERGY)
        self.assertAlmostEqual(r.turn_t, 321, delta=2)
        self.assertIsNotNone(r.home_t)
        self.assertEqual(self.link.commands_received, 1, "the drone acknowledged one RTL")
        texts = " ".join(e["text"] for e in self.link.monitor.events)
        self.assertIn("Pi: takeoff", texts)
        self.assertIn("Pi: landed", texts)

    def test_fault_injected_from_the_laptop(self):
        # A worn motor draws more current, so DroneGuard turns back early -
        # for energy or for health, whichever limit it hits first - and gets home.
        r = self.fly(faults=False, inject="motor")
        self.assertGreater(self.link.truth["motor"], 0.5, "the drone reports the injected fault")
        self.assertIsNotNone(r.warn_t)
        self.assertIn(r.turn_reason, (ABORT_ENERGY, ABORT_HEALTH))
        self.assertLess(r.turn_t, 321, "turns back earlier than the healthy-start reference")
        self.assertIsNotNone(r.home_t)


try:
    from fastapi.testclient import TestClient
    from web.app import app
except ImportError:
    app = None


@unittest.skipIf(app is None or live is None, "FastAPI or pymavlink not installed")
class LiveAPI(unittest.TestCase):

    def test_start_stream_fault_stop(self):
        client = TestClient(app)
        self.assertIn("DroneGuard Live", client.get("/live").text)
        self.assertEqual(client.post("/api/live/fault", json={"kind": "motor"}).status_code in (200, 409), True)
        st = client.post("/api/live/start", json={"speed": 100, "scenario": "clean"}).json()
        self.assertTrue(st["running"])
        with client.websocket_connect("/ws/live") as ws:
            got = 0
            deadline = time.time() + 10
            while got < 50 and time.time() < deadline:
                got += len(ws.receive_json().get("rows", []))
        self.assertGreaterEqual(got, 50)
        self.assertEqual(client.post("/api/live/fault", json={"kind": "battery"}).status_code, 200)
        self.assertEqual(client.post("/api/live/fault", json={"kind": "wings"}).status_code, 422)
        self.assertFalse(client.post("/api/live/stop").json()["running"])

    def test_planned_drone_and_spec_download(self):
        client = TestClient(app)
        q = "payload_kg=2&endurance_min=15&distance_km=3&cruise_mps=12&altitude_m=60"
        r = client.get(f"/api/spec?{q}")
        self.assertEqual(r.status_code, 200)
        self.assertIn("attachment", r.headers["content-disposition"])
        self.assertEqual(r.json()["thresholds"]["Range"], 3000)
        plan = dict(payload_kg=2, endurance_min=15, distance_km=3, cruise_mps=12, altitude_m=60)
        st = client.post("/api/live/start", json={"plan": plan, "speed": 100}).json()
        self.assertIn("Planned drone", st["label"])
        client.post("/api/live/stop")


if __name__ == "__main__":
    unittest.main()
