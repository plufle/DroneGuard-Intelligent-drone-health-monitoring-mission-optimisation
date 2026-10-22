"""The whole DroneGuard companion computer, in one call.

Port of matlab/src/droneguard_module.m: health first, then the mission decision,
with the alert passed from one to the other.
"""

from typing import NamedTuple

from .health import health
from .mission import MissionManager
from .thresholds import Thresholds


class ModuleOutput(NamedTuple):
    motor_health: float
    batt_health: float
    overall_health: float
    alert: int
    speed_cmd: float
    rth_threshold: float
    soc_needed: float
    mission_state: int


class DroneGuardModule:
    """Call step() once per tick (10 Hz) with the latest telemetry."""

    def __init__(self, c: Thresholds | None = None):
        self.c = c if c is not None else Thresholds()
        self.mission = MissionManager(self.c)

    def reset(self) -> None:
        self.mission.reset()

    def step(self, T_motor: float, I_motor: float, V_batt: float, soc: float,
             dist_home: float, use_dynamic: float = 1.0) -> ModuleOutput:
        h = health(T_motor, I_motor, V_batt, soc, self.c)
        m = self.mission.step(dist_home, soc, I_motor, use_dynamic, h.alert)
        return ModuleOutput(h.motor, h.battery, h.overall, h.alert, *m)
