"""Which drone to fly, and on which trip - built the same way on any machine.

The Raspberry Pi (playing the drone) and the laptop (running DroneGuard) both
call build_flight() with the same plan numbers, so the drone the Pi flies and
the limits DroneGuard watches it with always match.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from droneguard.check import Mission, to_sim
from droneguard.params import SimParams
from droneguard.sizing import Requirements, size
from droneguard.thresholds import Thresholds

PLAN_KEYS = ("payload_kg", "endurance_min", "distance_km", "cruise_mps", "altitude_m", "motors")


@dataclass
class Flight:
    p: SimParams
    c: Thresholds
    n_motors: int
    label: str
    fault_ramp_s: float      # how long an injected fault takes to reach full severity


def build_flight(plan: dict | None = None, faults: bool = False) -> Flight:
    """plan=None: the reference 2 kg drone on its 6 km route.
    plan={payload_kg, endurance_min, distance_km, cruise_mps, altitude_m, motors?}:
    the drone the sizing engine designs for that job."""
    if not plan:
        p = SimParams()
        if not faults:
            p = replace(p, MotorFault_Mag=0.0, BattFault_Mag=0.0)
        return Flight(p, Thresholds(), 4, "Reference drone: 2 kg, 4S 5000 mAh, 6 km route", 200.0)

    motors = plan.get("motors") or None
    d = size(Requirements(plan["payload_kg"] * 1000, plan["endurance_min"],
                          int(motors) if motors else None))
    if not d.converged:
        raise ValueError("that plan cannot be built: " + "; ".join(d.problems))
    mission = Mission(plan["distance_km"] * 1000, plan.get("cruise_mps", 12.0),
                      plan.get("altitude_m", 60.0))
    p, c = to_sim(d, mission, faults=faults)
    label = (f"Planned drone: {plan['payload_kg']:g} kg payload, {d.auw_g / 1000:.1f} kg, "
             f"{d.n_motors} motors, {d.cells}S {d.capacity_Ah * 1000:.0f} mAh, "
             f"{plan['distance_km']:g} km each way")
    ramp = max(60.0, 0.4 * 2 * mission.distance_m / mission.cruise_mps)
    return Flight(p, c, d.n_motors, label, ramp)
