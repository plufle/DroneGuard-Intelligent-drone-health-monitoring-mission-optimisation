"""Live monitoring (P4): the dashboard side.

    POST /api/live/start   start a virtual drone + DroneGuard monitor (UDP MAVLink)
    POST /api/live/stop
    POST /api/live/fault   {"kind": "motor" | "battery"} - wear it out mid-flight
    WS   /ws/live          10 updates a second: new telemetry rows, events, status
    GET  /live             the page

One session at a time; starting a new one replaces the old.
"""

import asyncio
import itertools
import time
from dataclasses import replace

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

import live  # noqa: F401  (forces MAVLink 2 before pymavlink loads)
from droneguard.check import Mission, to_sim
from droneguard.params import SimParams
from droneguard.sizing import Requirements, size
from droneguard.thresholds import Thresholds
from live.session import LiveSession

from .app import STATIC

router = APIRouter()
_session: LiveSession | None = None
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


class FaultRequest(BaseModel):
    kind: str


def _build(req: StartRequest):
    faults = req.scenario == "faults"
    if req.plan is None:
        p, c = SimParams(), Thresholds()
        if not faults:
            p = replace(p, MotorFault_Mag=0.0, BattFault_Mag=0.0)
        return p, c, 4, "Reference drone: 2 kg, 4S 5000 mAh, 6 km route", 200.0
    pl = req.plan
    d = size(Requirements(pl.payload_kg * 1000, pl.endurance_min, pl.motors))
    if not d.converged:
        raise HTTPException(400, "that plan cannot be built: " + "; ".join(d.problems))
    mission = Mission(pl.distance_km * 1000, pl.cruise_mps, pl.altitude_m)
    p, c = to_sim(d, mission, faults=faults)
    label = (f"Planned drone: {pl.payload_kg:g} kg payload, {d.auw_g / 1000:.1f} kg, "
             f"{d.n_motors} motors, {d.cells}S {d.capacity_Ah * 1000:.0f} mAh, "
             f"{pl.distance_km:g} km each way")
    ramp = max(60.0, 0.4 * 2 * mission.distance_m / mission.cruise_mps)
    return p, c, d.n_motors, label, ramp


@router.get("/live")
def live_page():
    return FileResponse(STATIC / "live.html")


@router.post("/api/live/start")
def start(req: StartRequest):
    global _session, _session_id
    if req.scenario not in ("clean", "faults") or req.rule not in ("dynamic", "fixed"):
        raise HTTPException(422, "scenario must be clean|faults, rule dynamic|fixed")
    p, c, n, label, ramp = _build(req)
    if _session is not None:
        _session.stop()
    _session = LiveSession(p, c, speed=req.speed, use_dynamic=float(req.rule == "dynamic"),
                           n_motors=n, label=label, fault_ramp_s=ramp)
    _session_id = next(_ids)
    _session.start()
    return _status()


@router.post("/api/live/stop")
def stop():
    if _session is not None:
        _session.stop()
    return _status()


@router.post("/api/live/fault")
def fault(req: FaultRequest):
    if _session is None or not _session.running:
        raise HTTPException(409, "no flight running")
    if req.kind not in ("motor", "battery"):
        raise HTTPException(422, "kind must be motor or battery")
    _session.inject(req.kind)
    return _status()


@router.get("/api/live/status")
def status():
    return _status()


def _status() -> dict:
    s = _session
    if s is None:
        return {"id": 0, "running": False}
    d, m, c = s.drone, s.monitor, s.c
    age = None if m.last_heard is None else round(time.monotonic() - m.last_heard, 2)
    return {
        "id": _session_id, "label": s.label, "running": s.running, "speed": s.speed,
        "t": round(d.t, 1), "drone_phase": d.phase_name,
        "faults": {k: round(v, 3) for k, v in d.fault_levels().items()},
        "messages": m.messages, "last_heard_s": age, "commands_received": s.commands_received,
        "rtl_sent_at": m.rtl_sent_at, "no_temperature": m.no_temperature,
        "range_m": c.Range, "reserve": c.Reserve, "warn": c.Warn, "alarm": c.Alarm,
        "port": s.port,
    }


@router.websocket("/ws/live")
async def ws_live(ws: WebSocket):
    await ws.accept()
    seen_id, row_i, ev_i = None, 0, 0
    try:
        while True:
            s = _session
            msg = {"status": _status()}
            if s is not None:
                if _session_id != seen_id:
                    seen_id, row_i, ev_i = _session_id, 0, 0
                    msg["reset"] = True
                with s.monitor.lock:
                    rows = s.monitor.rows[row_i:]
                    events = s.monitor.events[ev_i:]
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
