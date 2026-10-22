"""Live monitoring (P4): the dashboard side.

    POST /api/live/start      start a flight: on this laptop, or on the Raspberry Pi
    POST /api/live/stop
    POST /api/live/fault      {"kind": "motor" | "battery"} - wear it out mid-flight
    POST /api/live/pi/listen  start listening for the Pi on UDP 14550
    WS   /ws/live             10 updates a second: new telemetry rows, events, status
    GET  /live                the page

Either way DroneGuard runs HERE; the drone is either a thread on this laptop or
the Raspberry Pi (`python -m live.drone --serve --host <this laptop's IP>`).
One flight at a time; starting a new one replaces the old.
"""

from __future__ import annotations

import asyncio
import itertools
import time

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

import live  # noqa: F401  (forces MAVLink 2 before pymavlink loads)
from live.flights import build_flight
from live.remote import PiLink, lan_ip
from live.session import LiveSession

from .app import STATIC

PI_PORT = 14550
# Over Wi-Fi, faster than this and the laptop falls behind the Pi's telemetry:
# at 50x the RTL reached the Pi 15 s of flight late (home with 2%, not 10%).
PI_MAX_SPEED = 20.0

router = APIRouter()
_session = None                 # LiveSession (laptop) or PiLink (Pi)
_pi: PiLink | None = None       # kept open once started: the Pi stays connected
_ids = itertools.count(1)
_session_id = 0


class Plan(BaseModel):
    payload_kg: float = Field(gt=0, le=50)
    endurance_min: float = Field(gt=0, le=120)
    motors: int | None = None
    distance_km: float = Field(gt=0, le=30)
    cruise_mps: float = Field(12.0, ge=2, le=30)
    altitude_m: float = Field(60.0, ge=5, le=500)


class StartRequest(BaseModel):
    plan: Plan | None = None              # None = the reference 2 kg drone
    scenario: str = "clean"               # "clean" (inject yourself) or "faults" (scheduled)
    rule: str = "dynamic"                 # "dynamic" or "fixed"
    speed: float = Field(10.0, ge=0.5, le=200)
    source: str = "laptop"                # "laptop" or "pi"


class FaultRequest(BaseModel):
    kind: str


def _pi_link() -> PiLink:
    global _pi
    if _pi is None:
        try:
            _pi = PiLink(PI_PORT)
        except OSError as e:
            raise HTTPException(409, f"cannot listen on UDP {PI_PORT}: {e}")
    return _pi


@router.get("/live")
def live_page():
    return FileResponse(STATIC / "live.html")


@router.post("/api/live/pi/listen")
def pi_listen():
    _pi_link()
    return _status()


@router.post("/api/live/start")
def start(req: StartRequest):
    global _session, _session_id
    if req.scenario not in ("clean", "faults") or req.rule not in ("dynamic", "fixed") \
            or req.source not in ("laptop", "pi"):
        raise HTTPException(422, "scenario clean|faults, rule dynamic|fixed, source laptop|pi")
    plan = req.plan.model_dump() if req.plan else None
    faults = req.scenario == "faults"
    try:
        flight = build_flight(plan, faults)
    except ValueError as e:
        raise HTTPException(400, str(e))
    use_dynamic = float(req.rule == "dynamic")

    if _session is not None and _session is not _pi:
        _session.stop()                    # a laptop flight: end it
    if req.source == "pi":
        if req.speed > PI_MAX_SPEED:
            raise HTTPException(422, f"over Wi-Fi keep the speed at {PI_MAX_SPEED:g}x or less")
        pi = _pi_link()
        try:
            pi.start(flight, plan, faults, req.speed, use_dynamic)
        except RuntimeError as e:
            raise HTTPException(409, str(e))
        _session = pi
    else:
        if _session is _pi and _pi is not None:
            _pi.stop()                     # tell the Pi to land its flight
        _session = LiveSession(flight.p, flight.c, speed=req.speed, use_dynamic=use_dynamic,
                               n_motors=flight.n_motors, label=flight.label,
                               fault_ramp_s=flight.fault_ramp_s)
        _session.start()
    _session_id = next(_ids)
    return _status()


@router.post("/api/live/stop")
def stop():
    if _session is not None:
        _session.stop()
    return _status()


@router.post("/api/live/fault")
def fault(req: FaultRequest):
    if req.kind not in ("motor", "battery"):
        raise HTTPException(422, "kind must be motor or battery")
    if _session is None or not _session.running:
        raise HTTPException(409, "no flight running")
    _session.inject(req.kind)
    return _status()


@router.get("/api/live/status")
def status():
    return _status()


def _status() -> dict:
    s = _session
    out = {"id": _session_id, "running": False, "laptop_ip": lan_ip(), "pi_port": PI_PORT,
           "pi_listening": _pi is not None,
           "pi_connected": bool(_pi and _pi.connected),
           "pi_address": _pi.pi_address if _pi else None}
    if s is None or s.monitor is None:
        return out
    m, c = s.monitor, s.monitor.c
    age = None if m.last_heard is None else round(time.monotonic() - m.last_heard, 2)
    out.update(s.info())
    out.update({
        "label": s.label, "running": s.running, "speed": s.speed,
        "messages": m.messages, "last_heard_s": age,
        "rtl_sent_at": m.rtl_sent_at, "no_temperature": m.no_temperature,
        "range_m": c.Range, "reserve": c.Reserve, "warn": c.Warn, "alarm": c.Alarm,
    })
    return out


@router.websocket("/ws/live")
async def ws_live(ws: WebSocket):
    await ws.accept()
    seen_id, seen_monitor, row_i, ev_i = None, None, 0, 0
    try:
        while True:
            s = _session
            msg = {"status": _status()}
            mon = s.monitor if s is not None else None
            if mon is not None:
                if _session_id != seen_id or mon is not seen_monitor:
                    seen_id, seen_monitor, row_i, ev_i = _session_id, mon, 0, 0
                    msg["reset"] = True
                with mon.lock:
                    rows = mon.rows[row_i:]
                    events = mon.events[ev_i:]
                row_i += len(rows)
                ev_i += len(events)
                # t, dist, alt, speed, T, I, V, soc%, motor, batt, overall, alert, rth%, need%, state
                msg["rows"] = [[round(r[14], 1), round(r[2], 1), round(r[0], 1), round(r[1], 2),
                                round(r[3], 1), round(r[4], 2), round(r[5], 2), round(100 * r[6], 2),
                                round(r[7], 1), round(r[8], 1), round(r[9], 1), int(r[10]),
                                round(100 * r[11], 2), round(100 * r[12], 2), int(r[13])]
                               for r in rows]
                msg["events"] = events
            await ws.send_json(msg)
            await asyncio.sleep(0.1)
    except (WebSocketDisconnect, RuntimeError):
        pass
