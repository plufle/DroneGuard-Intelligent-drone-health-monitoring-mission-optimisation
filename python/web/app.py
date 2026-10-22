"""DroneGuard website: the planner (P3) and the live monitor (P4, web/live.py).

    cd python
    .venv/Scripts/python -m web          then open http://127.0.0.1:8000

Planner calls:
    GET  /api/applications   the presets and their slider ranges
    POST /api/plan           mission in -> parts list, cost, GO/NO-GO, graph data
    GET  /api/spec           the spec file for the Pi monitor (same inputs, as a query)
    GET  /                   the page itself (web/static/index.html)

Nothing is stored: every plan is computed from the inputs and thrown away.
"""

from dataclasses import asdict
from functools import lru_cache
from pathlib import Path

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from droneguard import applications
from droneguard.check import Mission, check, to_sim
from droneguard.sizing import Requirements, cost_inr_2022, size
from droneguard.summary import mission_end

STATIC = Path(__file__).parent / "static"
PLOT_POINTS = 400

app = FastAPI(title="DroneGuard planner")
app.mount("/static", StaticFiles(directory=STATIC), name="static")


class PlanRequest(BaseModel):
    application: str = "delivery"
    payload_kg: float = Field(2.0, gt=0, le=50)
    endurance_min: float = Field(15.0, gt=0, le=120)
    motors: int | None = Field(None, description="4, 6, 8, or null for the lightest")
    distance_km: float = Field(3.0, gt=0, le=30)
    cruise_mps: float = Field(12.0, ge=2, le=30)
    altitude_m: float = Field(60.0, ge=5, le=500)


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/applications")
def list_applications():
    return [asdict(a) for a in applications.APPLICATIONS]


@app.post("/api/plan")
def plan(req: PlanRequest):
    try:
        app_ = applications.get(req.application)
    except KeyError:
        raise HTTPException(404, f"unknown application '{req.application}'")
    if not app_.available:
        raise HTTPException(400, f"{app_.name} is not supported yet")
    if req.motors not in (None, 4, 6, 8):
        raise HTTPException(422, "motors must be 4, 6, 8 or null")
    return _plan(req.payload_kg, req.endurance_min, req.motors,
                 req.distance_km, req.cruise_mps, req.altitude_m)


@app.get("/api/spec")
def spec(payload_kg: float = 2.0, endurance_min: float = 15.0, motors: int | None = None,
         distance_km: float = 3.0, cruise_mps: float = 12.0, altitude_m: float = 60.0):
    """The one spec file for the Pi monitor: the limits this design was checked with."""
    from live.spec import spec_dict
    d = size(Requirements(payload_kg * 1000, endurance_min, motors))
    if not d.converged:
        raise HTTPException(400, "that plan cannot be built: " + "; ".join(d.problems))
    _, c = to_sim(d, Mission(distance_km * 1000, cruise_mps, altitude_m))
    body = spec_dict(c, design=_design(d),
                     mission={"distance_km": distance_km, "cruise_mps": cruise_mps,
                              "altitude_m": altitude_m})
    name = f"droneguard_{payload_kg:g}kg_{distance_km:g}km.json"
    return JSONResponse(body, headers={"Content-Disposition": f'attachment; filename="{name}"'})


@lru_cache(maxsize=256)
def _plan(payload_kg, endurance_min, motors, distance_km, cruise_mps, altitude_m) -> dict:
    d = size(Requirements(payload_kg * 1000, endurance_min, motors))
    out = {"design": _design(d), "cost_inr": cost_inr_2022(payload_kg * 1000),
           "check": None, "series": None}
    if not d.converged:
        return out
    r = check(d, Mission(distance_km * 1000, cruise_mps, altitude_m))
    out["check"] = {
        "verdict": r.verdict,
        "reasons": r.reasons,
        "cautions": r.cautions,
        "healthy": _summary(r.healthy),
        "stressed": _summary(r.stressed),
        "max_distance_km": None if r.max_distance_m is None else r.max_distance_m / 1000,
    }
    out["series"] = {name: _series(r.runs[name]) for name in ("healthy", "stressed")}
    return out


def _design(d) -> dict:
    out = {"feasible": d.feasible, "converged": d.converged, "problems": d.problems,
           "n_motors": d.n_motors}
    if not d.converged:
        return out
    m = d.motor
    out.update(
        auw_kg=d.auw_g / 1000,
        passes=len(d.iterations),
        motor={"name": m.name, "kv": m.kv, "prop": m.prop.replace("*", "x"),
               "T_max_g": m.T_max_g, "I_max": m.I_max, "V_test": m.V_test},
        esc_A=d.esc_A,
        battery={"cells": d.cells, "mAh": round(d.capacity_Ah * 1000), "C": d.c_rating,
                 "V_nominal": d.V_nominal},
        thrust_to_weight=d.thrust_to_weight,
        hover={"per_motor_g": d.hover_thrust_g, "current_A": d.I_hover_total,
               "power_W": d.hover_power_W},
        full_throttle={"current_A": d.I_max_total, "power_W": d.I_max_total * d.V_nominal},
        endurance_min=d.endurance_min,
        masses_g=d.masses_g,
    )
    return out


def _summary(s) -> dict:
    return {k: v for k, v in asdict(s).items()}


def _series(r) -> dict:
    """The run trimmed to where the mission ended, thinned for drawing."""
    e = mission_end(r.mission_state, r.soc) + 1
    idx = np.unique(np.linspace(0, e - 1, min(PLOT_POINTS, e)).round().astype(int))

    def col(a, scale=1.0, nd=3):
        return np.round(a[idx] * scale, nd).tolist()

    return {"t": col(r.t, nd=1), "dist_km": col(r.dist, 1e-3), "soc": col(r.soc, 100, 2),
            "rth": col(r.rth_threshold, 100, 2), "overall": col(r.overall_h, nd=2),
            "motor": col(r.motor_h, nd=2), "battery": col(r.batt_h, nd=2),
            "current_A": col(r.I_motor, nd=2), "state": col(r.mission_state, nd=0)}


# The live monitor needs pymavlink; without it the planner still works.
try:
    from .live import router as _live_router
except ImportError as _e:                      # pragma: no cover
    print(f"live monitor disabled ({_e}); pip install -r requirements.txt")
else:
    app.include_router(_live_router)
