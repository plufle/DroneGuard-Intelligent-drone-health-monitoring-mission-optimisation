"""Turn a simulation into the facts a person cares about.

Same rules as matlab/sim/droneguard_missionend.m and the console summary at the
end of matlab/sim/droneguard_plot.m.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sim import SimResult
from .thresholds import Thresholds

COMPLETE = "MISSION COMPLETE - reached the waypoint"
ABORT_ENERGY = "ABORT - no longer enough charge to get home"
ABORT_HEALTH = "ABORT - health ALARM, not fit to continue"


def _first(mask: np.ndarray) -> int | None:
    idx = np.flatnonzero(mask)
    return int(idx[0]) if idx.size else None


def mission_end(mission_state: np.ndarray, soc: np.ndarray) -> int:
    """Index where the mission is over: home, or the pack hit empty."""
    candidates = [i for i in (_first(mission_state >= 2), _first(soc <= 0))
                  if i is not None]
    return min(candidates + [len(soc) - 1])


@dataclass
class Summary:
    turn_t: float | None = None
    turn_dist: float | None = None
    turn_soc: float | None = None
    turn_reason: str | None = None
    home_t: float | None = None
    home_soc: float | None = None
    dead_t: float | None = None
    dead_dist: float | None = None
    end_dist: float = 0.0
    end_overall: float = 0.0
    cruise_current: float = 0.0
    peak_current: float = 0.0
    peak_temp: float = 0.0
    warn_t: float | None = None
    alarm_t: float | None = None

    def text(self, c: Thresholds | None = None) -> str:
        c = c if c is not None else Thresholds()
        lines = ["--- DroneGuard mission summary ---",
                 f"Waypoint            : {c.Range:.0f} m out"]
        if self.turn_t is not None:
            lines.append(f"Turned back         : t = {self.turn_t:.0f} s at "
                         f"{self.turn_dist:.0f} m  ({self.turn_reason})")
        if self.home_t is not None:
            lines.append(f"Outcome             : HOME at t = {self.home_t:.0f} s "
                         f"with {100 * self.home_soc:.0f}% charge left")
        elif self.dead_t is not None:
            lines.append(f"Outcome             : BATTERY DEAD at t = {self.dead_t:.0f} s, "
                         f"{self.dead_dist:.0f} m from home")
        else:
            lines.append(f"Outcome             : still flying when the run ended, "
                         f"{self.end_dist:.0f} m out")
        lines.append(f"Cruise current      : {self.cruise_current:.1f} A -> "
                     f"{self.peak_current:.1f} A")
        lines.append(f"Peak motor temp     : {self.peak_temp:.0f} C")
        if self.warn_t is not None:
            lines.append(f"WARN  raised at     : {self.warn_t:.0f} s")
        if self.alarm_t is not None:
            lines.append(f"ALARM raised at     : {self.alarm_t:.0f} s")
        return "\n".join(lines)


def summarize(r: SimResult, c: Thresholds | None = None) -> Summary:
    c = c if c is not None else Thresholds()
    i_end = mission_end(r.mission_state, r.soc)
    sl = slice(0, i_end + 1)
    t, dist, soc = r.t[sl], r.dist[sl], r.soc[sl]
    state, alert, rth = r.mission_state[sl], r.alert[sl], r.rth_threshold[sl]

    s = Summary(end_dist=float(dist[-1]), end_overall=float(r.overall_h[i_end]),
                cruise_current=float(r.I_motor[0]),
                peak_current=float(r.I_motor[sl].max()),
                peak_temp=float(r.T_motor[sl].max()))

    i_turn = _first(state >= 1)
    if i_turn is not None:
        s.turn_t, s.turn_dist = float(t[i_turn]), float(dist[i_turn])
        s.turn_soc = float(soc[i_turn])
        # Three different things can end the outbound leg.
        if dist[i_turn] >= c.Range - 5:
            s.turn_reason = COMPLETE
        elif soc[i_turn] <= rth[i_turn] + 1e-6:
            s.turn_reason = ABORT_ENERGY
        elif alert[i_turn] >= 2:
            s.turn_reason = ABORT_HEALTH
        else:
            s.turn_reason = "reason unclear"

    i_home = _first(state >= 2)
    if i_home is not None:
        s.home_t, s.home_soc = float(t[i_home]), float(soc[i_home])
    else:
        i_dead = _first(soc <= 0)
        if i_dead is not None:
            s.dead_t, s.dead_dist = float(t[i_dead]), float(dist[i_dead])

    i_warn, i_alarm = _first(alert >= 1), _first(alert >= 2)
    s.warn_t = float(t[i_warn]) if i_warn is not None else None
    s.alarm_t = float(t[i_alarm]) if i_alarm is not None else None
    return s
