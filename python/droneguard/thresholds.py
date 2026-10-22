"""Single source of truth for the DroneGuard module: health limits AND mission rules.

Mirrors matlab/src/droneguard_thresholds.m number for number. Field names are kept
identical to the MATLAB struct so the two can be compared side by side.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Thresholds:
    # ---- health score limits ----
    T_good: float = 60.0      # [degC] motor temperature still healthy
    T_bad: float = 100.0      # [degC] motor temperature critical
    I_good: float = 28.0      # [A]    motor current still healthy
    I_bad: float = 40.0       # [A]    motor current critical
    Sag_good: float = 0.10    # [V]    voltage sag beyond prediction - healthy
    Sag_bad: float = 1.50     # [V]    voltage sag beyond prediction - critical
    Warn: float = 70.0        # [-]    below this -> caution
    Alarm: float = 40.0       # [-]    below this -> abort / return to home
    # ---- what a healthy pack looks like ----
    V_full: float = 16.8      # [V]    open-circuit voltage at 100% SOC
    V_empty: float = 14.0     # [V]    open-circuit voltage at 0% SOC
    R_int: float = 0.020      # [Ohm]  healthy internal resistance
    Cap_Ah: float = 5.0       # [Ah]   rated pack capacity
    # ---- mission rules ----
    Range: float = 6000.0     # [m]    how far out the mission goes before turning
    Cruise: float = 12.0      # [m/s]  commanded cruise speed
    Reserve: float = 0.10     # [-]    SOC we refuse to land below
    Ts: float = 0.1           # [s]    rate this module runs at
    RTH_margin: float = 1.25  # [-]    safety factor on the energy-to-home estimate
    Trend_tau: float = 20.0   # [s]    smoothing on the current-growth estimate
    RTH_fixed: float = 0.25   # [-]    the naive "always turn back at 25%" rule
