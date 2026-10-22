"""Component and overall drone health scores (0..100).

Port of matlab/src/droneguard_health.m. No state, no dependencies - this is the
part that runs on the Raspberry Pi.
"""

from __future__ import annotations

from typing import NamedTuple

from .thresholds import Thresholds

OK, WARN, ALARM = 0, 1, 2


class Health(NamedTuple):
    motor: float     # 0..100
    battery: float   # 0..100
    overall: float   # 0..100, the worse of the two
    alert: int       # 0 = OK, 1 = WARN, 2 = ALARM


def lin_score(x: float, good: float, bad: float) -> float:
    """Map an indicator onto 100 (at good) .. 0 (at bad), clamped.

    Works whether "bad" is a higher or a lower number than "good".
    """
    s = 100.0 * (bad - x) / (bad - good)
    return min(max(s, 0.0), 100.0)


def health(T_motor: float, I_motor: float, V_batt: float, soc: float,
           p: Thresholds) -> Health:
    # Motor: running hot and pulling too much current are two symptoms of the
    # same wear, so score each one and keep the worse.
    h_temp = lin_score(T_motor, p.T_good, p.T_bad)
    h_current = lin_score(I_motor, p.I_good, p.I_bad)
    motor = min(h_temp, h_current)

    # Battery: voltage sag beyond what the SOC predicts means internal
    # resistance has risen - the classic ageing signature.
    V_expected = p.V_empty + (p.V_full - p.V_empty) * soc - I_motor * p.R_int
    sag = V_expected - V_batt
    battery = lin_score(sag, p.Sag_good, p.Sag_bad)

    # min(), NOT mean(): averaging would let a healthy battery hide a dying motor.
    overall = min(motor, battery)

    if overall < p.Alarm:
        alert = ALARM
    elif overall < p.Warn:
        alert = WARN
    else:
        alert = OK
    return Health(motor, battery, overall, alert)
