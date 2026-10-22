"""Mission optimisation: decide when to turn back.

Port of matlab/src/droneguard_mission.m. The MATLAB version keeps its memory in
`persistent` variables; here that memory lives on a MissionManager object, so
two simulations can run side by side without sharing state.
"""

from __future__ import annotations

from typing import NamedTuple

from .health import ALARM
from .thresholds import Thresholds

OUTBOUND, RETURNING, HOME = 0, 1, 2


class MissionOutput(NamedTuple):
    speed_cmd: float       # [m/s] forward speed command (negative = flying home)
    rth_threshold: float   # [-]   SOC at which we must turn back
    soc_needed: float      # [-]   charge the trip home would actually cost
    mission_state: int     # 0 = outbound, 1 = returning, 2 = home


class MissionManager:
    """Answers one question at every tick: "if I turn round now, can I get home?" """

    def __init__(self, c: Thresholds):
        self.c = c
        self.reset()

    def reset(self) -> None:
        self.state = None          # set on the first call, like MATLAB's isempty(state)
        self.I_prev = 0.0
        self.dI_smooth = 0.0

    def step(self, dist_home: float, soc: float, I_motor: float,
             use_dynamic: float, alert: int) -> MissionOutput:
        c = self.c
        if self.state is None:
            self.state = OUTBOUND
            self.I_prev = I_motor
            self.dI_smooth = 0.0

        # What would the trip home cost? Project the current draw forward,
        # because a degrading motor keeps getting worse all the way home.
        # Only upward trends count.
        d_home = max(dist_home, 0.0)
        t_home = d_home / c.Cruise

        dI_raw = (I_motor - self.I_prev) / c.Ts
        alpha = c.Ts / c.Trend_tau
        self.dI_smooth = self.dI_smooth + alpha * (dI_raw - self.dI_smooth)
        self.I_prev = I_motor

        I_expected = I_motor + 0.5 * max(self.dI_smooth, 0.0) * t_home
        Ah_home = I_expected * t_home / 3600.0
        soc_needed = (Ah_home / c.Cap_Ah) * c.RTH_margin

        if use_dynamic > 0.5:
            rth_threshold = soc_needed + c.Reserve
        else:
            rth_threshold = c.RTH_fixed   # the naive rule, blind to the drone's health

        # State machine: three independent reasons to stop heading away.
        if self.state == OUTBOUND:
            reached_waypoint = dist_home >= c.Range
            out_of_margin = soc <= rth_threshold
            unfit_to_continue = alert >= ALARM
            if reached_waypoint or out_of_margin or unfit_to_continue:
                self.state = RETURNING
        elif self.state == RETURNING:
            if dist_home <= 1.0:
                self.state = HOME

        if self.state == OUTBOUND:
            speed_cmd = c.Cruise
        elif self.state == RETURNING:
            speed_cmd = -c.Cruise
        else:
            speed_cmd = 0.0

        return MissionOutput(speed_cmd, rth_threshold, soc_needed, self.state)
