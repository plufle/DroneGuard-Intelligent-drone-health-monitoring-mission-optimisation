"""Design check: fly a sized drone on its real mission before anyone buys parts.

Takes a Design from the sizing engine and a mission (how far, how fast, how
high), turns them into simulation parameters, and flies three missions:

    healthy   - no faults: can it fly out and back with charge to spare?
    stressed  - the reference faults, timed so both are at full severity when
                the drone is due home (motor wear from 19% of the planned
                flight, battery ageing from halfway): does DroneGuard get it home?
    range     - no faults, waypoint set impossibly far: how far CAN it go?

Verdict: GO when the healthy mission completes with the reserve left and the
stressed mission still lands at home. Otherwise NO-GO, with the reason.

Every number that is not in the design is scaled from the reference 2 kg drone
(matlab/sim/droneguard_params.m), so a design identical to the reference maps
back onto exactly the reference parameters.

    python -m droneguard.check --payload-kg 2 --endurance-min 15 --distance-km 3
"""

import argparse
from dataclasses import dataclass, replace

from . import catalogue as cat
from .params import SimParams
from .sim import SimResult, simulate
from .sizing import Design, Requirements, size
from .summary import COMPLETE, Summary, summarize
from .thresholds import Thresholds

_REF_P, _REF_C = SimParams(), Thresholds()
_REF_MASS_KG = _REF_P.m_drone
_REF_I_HOVER = _REF_P.m_drone * _REF_P.g_accel / _REF_P.k_thrust       # 18.0 A
_REF_CELLS = round(_REF_C.V_full / cat.CELL_V_FULL)                     # 4S
_R_CELL_AH = _REF_C.R_int * _REF_C.Cap_Ah / _REF_CELLS                  # ohm*Ah per cell


@dataclass(frozen=True)
class Mission:
    distance_m: float            # one way, home to the destination
    cruise_mps: float = 12.0
    altitude_m: float = 50.0
    reserve: float = 0.10        # charge that must be left on landing


def to_sim(d: Design, mission: Mission, faults: bool = False,
           T_end: float | None = None,
           fault_by: float | None = None) -> tuple[SimParams, Thresholds]:
    """Design + mission -> the numbers the simulation and the module run on.

    With faults=True both faults reach full severity at `fault_by` seconds -
    by default the time the drone is due home, so the stress test bites
    whatever the mission length. They start at the same fractions of that time
    as in the reference (motor 150/800, battery 400/800)."""
    if not d.converged:
        raise ValueError("this design did not size; nothing to fly")
    m = d.auw_g / 1000.0
    g = _REF_P.g_accel
    scale_m = m / _REF_MASS_KG                       # same closed-loop response at any mass
    I_hover = d.I_hover_total
    i_ratio = I_hover / _REF_I_HOVER
    v_ratio = d.cells / _REF_CELLS

    t_plan = 2 * mission.distance_m / mission.cruise_mps     # out and back
    if T_end is None:   # long enough to fly out and back, with room for an abort
        T_end = 1.5 * t_plan + 60
    if fault_by is None:
        fault_by = t_plan
    m_start = fault_by * _REF_P.MotorFault_Start / _REF_P.T_end
    b_start = fault_by * _REF_P.BattFault_Start / _REF_P.T_end

    c = replace(
        _REF_C,
        I_good=_REF_C.I_good * i_ratio,
        I_bad=_REF_C.I_bad * i_ratio,
        Sag_good=_REF_C.Sag_good * v_ratio,
        Sag_bad=_REF_C.Sag_bad * v_ratio,
        V_full=d.cells * cat.CELL_V_FULL,
        V_empty=d.cells * cat.CELL_V_EMPTY,
        R_int=_R_CELL_AH * d.cells / d.capacity_Ah,
        Cap_Ah=d.capacity_Ah,
        Range=mission.distance_m,
        Cruise=mission.cruise_mps,
        Reserve=mission.reserve,
    )
    p = replace(
        _REF_P,
        T_end=T_end,
        Alt_cmd=mission.altitude_m,
        m_drone=m,
        Cd_forward=_REF_P.Cd_forward * scale_m ** (2 / 3),   # drag ~ frontal area
        Kp_alt=_REF_P.Kp_alt * scale_m, Ki_alt=_REF_P.Ki_alt * scale_m,
        Kd_alt=_REF_P.Kd_alt * scale_m,
        Kp_spd=_REF_P.Kp_spd * scale_m, Ki_spd=_REF_P.Ki_spd * scale_m,
        k_thrust=m * g / I_hover,                    # draws the design's hover current
        K_therm=_REF_P.K_therm / i_ratio ** 2,       # same temperature rise in hover
        V_full=c.V_full, V_empty=c.V_empty, R_int=c.R_int, Cap_Ah=c.Cap_Ah,
        MotorFault_Start=m_start,
        BattFault_Start=b_start,
        MotorFault_Mag=1.0 if faults else 0.0,
        BattFault_Mag=1.0 if faults else 0.0,
        MotorFault_Slope=(1.0 / (fault_by - m_start)) if faults else 0.0,
        BattFault_Slope=(1.0 / (fault_by - b_start)) if faults else 0.0,
    )
    return p, c


@dataclass
class Check:
    design: Design
    mission: Mission
    healthy: Summary
    stressed: Summary
    max_distance_m: float | None
    runs: dict                   # name -> SimResult, for plotting
    reasons: list                # why it is NO-GO
    cautions: list               # GO, but with little margin

    @property
    def go(self) -> bool:
        return not self.reasons

    @property
    def verdict(self) -> str:
        if self.reasons:
            return "NO-GO"
        return "GO WITH CAUTION" if self.cautions else "GO"

    def text(self) -> str:
        ms, h, s = self.mission, self.healthy, self.stressed
        lines = [f"--- DroneGuard design check: {ms.distance_m / 1000:g} km out and back "
                 f"at {ms.cruise_mps:g} m/s, {ms.altitude_m:g} m up ---"]

        def outcome(x: Summary) -> str:
            turned = (f"turned at {x.turn_dist:.0f} m, t = {x.turn_t:.0f} s "
                      f"({x.turn_reason})" if x.turn_t is not None else "never turned")
            if x.home_t is not None:
                end = f"home at {x.home_t:.0f} s with {100 * x.home_soc:.0f}% charge"
            elif x.dead_t is not None:
                end = f"BATTERY DEAD {x.dead_dist:.0f} m from home"
            else:
                end = "still flying when the run ended"
            return f"{turned}; {end}"

        lines.append(f"Healthy drone       : {outcome(h)}")
        lines.append(f"Worn motor + old pack: {outcome(s)}")
        alerts = [f"{name} at {t:.0f} s" for name, t in (("WARN", s.warn_t), ("ALARM", s.alarm_t))
                  if t is not None]
        lines.append("  DroneGuard health  : " + (", ".join(alerts) if alerts else
                                                  "no alerts during the flight")
                     + f"; health at landing {s.end_overall:.0f}/100")
        if self.max_distance_m is not None:
            lines.append(f"Furthest it can go  : about {self.max_distance_m / 1000:.1f} km "
                         f"out and back (healthy, keeping {100 * ms.reserve:.0f}% reserve)")
        lines.append(f"VERDICT             : {self.verdict}")
        lines += [f"  - {r}" for r in self.reasons + self.cautions]
        return "\n".join(lines)


def check(d: Design, mission: Mission) -> Check:
    runs: dict[str, SimResult] = {}
    summaries = {}
    for name, faults in (("healthy", False), ("stressed", True)):
        p, c = to_sim(d, mission, faults)
        runs[name] = simulate(p, c)
        summaries[name] = summarize(runs[name], c)

    # Range: same healthy drone, waypoint out of reach, plenty of time.
    full_pack_s = d.capacity_Ah / d.I_hover_total * 3600
    far = replace(mission, distance_m=10 * full_pack_s * mission.cruise_mps)
    p, c = to_sim(d, far, faults=False, T_end=1.2 * full_pack_s)
    runs["range"] = simulate(p, c)
    r = summarize(runs["range"], c)
    max_distance = r.turn_dist if r.home_t is not None else None

    h, s = summaries["healthy"], summaries["stressed"]
    reasons = []
    if h.turn_reason != COMPLETE:
        reasons.append("a HEALTHY drone cannot reach the destination: it had to turn "
                       f"back {h.turn_dist:.0f} m out - bigger battery or shorter trip")
    elif h.home_t is None:
        reasons.append("a healthy drone reaches the destination but does not get home")
    elif h.home_soc < mission.reserve - 0.005:
        reasons.append(f"lands with {100 * h.home_soc:.0f}% charge, below the "
                       f"{100 * mission.reserve:.0f}% reserve")
    cautions = []
    if s.home_t is None:
        reasons.append("with a worn motor and an aged pack it does NOT get home, "
                       "even with DroneGuard turning it back")
    elif s.home_soc < mission.reserve - 0.005:
        cautions.append(f"worn out, it lands with only {100 * s.home_soc:.0f}% charge - "
                        "little margin for wind or a weak pack")
    return Check(d, mission, h, s, max_distance, runs, reasons, cautions)


def main():
    ap = argparse.ArgumentParser(prog="python -m droneguard.check")
    ap.add_argument("--payload-kg", type=float, default=2.0)
    ap.add_argument("--endurance-min", type=float, default=15.0)
    ap.add_argument("--motors", type=int, choices=(4, 6, 8))
    ap.add_argument("--distance-km", type=float, default=3.0, help="one way")
    ap.add_argument("--cruise", type=float, default=12.0, help="m/s")
    ap.add_argument("--altitude", type=float, default=50.0, help="m")
    a = ap.parse_args()

    d = size(Requirements(a.payload_kg * 1000, a.endurance_min, a.motors))
    print(d.text())
    if not d.converged:
        return
    print()
    print(check(d, Mission(a.distance_km * 1000, a.cruise, a.altitude)).text())


if __name__ == "__main__":
    main()
