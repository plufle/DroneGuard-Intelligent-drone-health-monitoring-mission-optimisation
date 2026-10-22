"""What each kind of drone job asks for - the presets the planning website offers.

Numbers come from Section 2 of the component-selection paper where it gives
them, and are marked where they do not. Delivery is the first application the
planner supports; the others are listed so the website can show what is next.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Range:
    lo: float
    hi: float
    step: float
    default: float
    unit: str


@dataclass(frozen=True)
class Application:
    key: str
    name: str
    available: bool
    summary: str
    payload_kg: Range | None = None
    endurance_min: Range | None = None
    distance_km: Range | None = None
    cruise_mps: Range | None = None
    altitude_m: Range | None = None
    notes: tuple[str, ...] = ()


DELIVERY = Application(
    key="delivery",
    name="Delivery",
    available=True,
    summary="Carry a parcel to a drop point and fly back to base.",
    payload_kg=Range(0.5, 10, 0.5, 2, "kg"),
    endurance_min=Range(5, 45, 1, 15, "min"),
    distance_km=Range(0.5, 10, 0.1, 3, "km"),
    cruise_mps=Range(5, 20, 1, 12, "m/s"),
    altitude_m=Range(20, 120, 5, 60, "m"),
    notes=(
        "Paper: an octocopter carrying 6 kg for 45 minutes, up to a 5 km radius. "
        "With realistic part weights that is not possible with its parts list - "
        "try it.",
        "120 m is the ceiling for green-zone flights under India's Drone Rules 2021.",
        "Hover time is used to size the battery; the design check then flies the "
        "real trip, including drag at cruise speed.",
    ),
)

APPLICATIONS = (
    DELIVERY,
    Application("firefighting", "Firefighting", False,
                "Eight motors, a 1.5 kg extinguisher ball, thermal camera and lights.",
                notes=("The paper's 6S 1200 mAh 15C pack cannot feed eight motors.",)),
    Application("agriculture", "Agriculture", False,
                "10-15 L spray tank, eight nozzles, about 10 acres an hour.",
                notes=("Heavy lift: beyond the strongest motor in the paper's catalogue "
                       "once real weights are counted.",)),
    Application("healthcare", "Healthcare (VTOL)", False,
                "Fixed-wing VTOL, 2 kg at 100 km/h over 160 km.",
                notes=("Fixed-wing is out of scope: DroneGuard models multirotors.",)),
)


def get(key: str) -> Application:
    for a in APPLICATIONS:
        if a.key == key:
            return a
    raise KeyError(key)
