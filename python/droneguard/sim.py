"""The DroneGuard_Sim.slx model, step for step, in plain Python.

Same blocks, same equations, same solver (fixed-step ode3 = Bogacki-Shampine at
Ts), same order of operations each step:

    1. Command_Delay outputs last tick's speed command
    2. Speed_Slew_Limit moves the command at most Accel_max*Ts towards it
    3. the plant's outputs (current, temperature, voltage, ...) are read
    4. the DroneGuard module runs on those readings
    5. the row is logged and the delay stores the new speed command
    6. the continuous states are integrated to the next tick, with the
       speed command held constant over the step (zero-order hold)

Continuous states (9): altitude, vertical rate, forward speed, distance,
altitude-PID integrator and derivative filter, speed-PI integrator, motor
thermal lag, battery SOC. Plus the Command_Delay = the model's 10 states.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .module import DroneGuardModule
from .params import SimParams
from .thresholds import Thresholds

# simData column order, identical to the Mux_Log block in the Simulink model.
COLUMNS = ("alt", "speed", "dist", "T_motor", "I_motor", "V_batt", "soc",
           "motor_h", "batt_h", "overall_h", "alert", "rth_threshold",
           "soc_needed", "mission_state")

# Indices into the continuous state vector.
Z, VZ, V, X, I_ALT, F_ALT, I_SPD, TM, SOC = range(9)


@dataclass
class SimResult:
    t: np.ndarray        # (n,) time [s]
    data: np.ndarray     # (n, 14) same columns as MATLAB simData
    params: SimParams

    def __getattr__(self, name):
        if name in COLUMNS:
            return self.data[:, COLUMNS.index(name)]
        raise AttributeError(name)


def _ramp(t: float, start: float, slope: float) -> float:
    """Simulink Ramp block followed by a 0..1 Saturation block."""
    if t < start:
        return 0.0
    return min(max(slope * (t - start), 0.0), 1.0)


class Plant:
    """Flight controller + dynamics + propulsion + battery, as continuous ODEs."""

    def __init__(self, p: SimParams):
        self.p = p
        self.m_slope = p.motor_slope
        self.b_slope = p.batt_slope

    def initial_state(self) -> list[float]:
        p = self.p
        y = [0.0] * 9
        y[Z] = p.alt_init
        y[V] = p.Speed_init
        y[I_SPD] = p.Cd_forward * p.Speed_init   # PI starts trimmed for Speed_init
        y[SOC] = p.SOC_init
        return y

    def signals(self, t: float, y: list[float], v_cmd: float):
        """Everything the blocks compute from the states, without integrating."""
        p = self.p
        # FlightController: PID on altitude (+ hover feedforward), PI on speed.
        e_alt = p.Alt_cmd - y[Z]
        d_alt = p.N_alt * (p.Kd_alt * e_alt - y[F_ALT])
        T_vert = p.Kp_alt * e_alt + y[I_ALT] + d_alt + p.m_drone * p.g_accel
        e_spd = v_cmd - y[V]
        T_fwd = p.Kp_spd * e_spd + y[I_SPD]

        # Propulsion: worn motors give less thrust per amp, so the same thrust
        # costs more current. The fault never touches the current directly.
        fault_m = _ramp(t, p.MotorFault_Start, self.m_slope)
        thrust = math.hypot(T_vert, T_fwd)
        I_motor = thrust / (p.k_thrust * (1.0 - p.k_eff_fault * fault_m))

        # Battery: an aged pack has higher resistance and drains faster.
        fault_b = _ramp(t, p.BattFault_Start, self.b_slope)
        soc = y[SOC]
        V_batt = (p.V_empty + (p.V_full - p.V_empty) * soc
                  - I_motor * p.R_int * (1.0 + p.k_R_fault * fault_b))
        return e_alt, d_alt, T_vert, e_spd, T_fwd, I_motor, fault_b, V_batt

    def derivatives(self, t: float, y: list[float], v_cmd: float) -> list[float]:
        p = self.p
        e_alt, d_alt, T_vert, e_spd, T_fwd, I_motor, fault_b, _ = \
            self.signals(t, y, v_cmd)
        m, g = p.m_drone, p.g_accel

        dsoc = -I_motor / (3600.0 * p.Cap_Ah) * (1.0 + p.k_drain_fault * fault_b)
        # Saturating integrator: stop at the limits instead of pushing past them.
        if (y[SOC] <= 0.0 and dsoc < 0.0) or (y[SOC] >= 1.0 and dsoc > 0.0):
            dsoc = 0.0

        return [
            y[VZ],                                       # altitude
            (T_vert - m * g) / m,                        # vertical rate
            (T_fwd - p.Cd_forward * y[V]) / m,           # forward speed
            y[V],                                        # distance
            p.Ki_alt * e_alt,                            # altitude PID integrator
            d_alt,                                       # altitude PID D-filter
            p.Ki_spd * e_spd,                            # speed PI integrator
            (p.K_therm * I_motor ** 2 - y[TM]) / p.tau_therm,   # thermal lag
            dsoc,                                        # battery SOC
        ]

    def step_ode3(self, t: float, y: list[float], v_cmd: float, h: float) -> list[float]:
        """One fixed step of Simulink's ode3 (Bogacki-Shampine)."""
        f = self.derivatives
        k1 = f(t, y, v_cmd)
        k2 = f(t + h / 2, [a + h / 2 * b for a, b in zip(y, k1)], v_cmd)
        k3 = f(t + 3 * h / 4, [a + 3 * h / 4 * b for a, b in zip(y, k2)], v_cmd)
        y_new = [a + h * (2 / 9 * b1 + 1 / 3 * b2 + 4 / 9 * b3)
                 for a, b1, b2, b3 in zip(y, k1, k2, k3)]
        y_new[SOC] = min(max(y_new[SOC], 0.0), 1.0)
        return y_new


def simulate(p: SimParams | None = None, c: Thresholds | None = None) -> SimResult:
    """Fly one mission. Equivalent to sim('DroneGuard_Sim') after droneguard_params."""
    p = p if p is not None else SimParams()
    c = c if c is not None else Thresholds()
    plant = Plant(p)
    module = DroneGuardModule(c)

    h = p.Ts
    n = int(round(p.T_end / h)) + 1
    t_log = np.arange(n) * h
    data = np.empty((n, len(COLUMNS)))

    y = plant.initial_state()
    delay = 0.0          # Command_Delay state (Unit Delay, initial 0)
    v_cmd = 0.0          # Speed_Slew_Limit output
    max_step = p.Accel_max * h

    for k in range(n):
        t = k * h
        # 1-2. delayed command through the slew-rate limiter
        if k > 0:
            v_cmd += min(max(delay - v_cmd, -max_step), max_step)
        # 3. read the plant
        _, _, _, _, _, I_motor, _, V_batt = plant.signals(t, y, v_cmd)
        T_motor = y[TM] + p.T_ambient
        # 4. DroneGuard
        out = module.step(T_motor, I_motor, V_batt, y[SOC], y[X], p.Use_Dynamic_RTH)
        # 5. log (Mux_Log skips speed_cmd), then latch the command for the next tick
        data[k] = (y[Z], y[V], y[X], T_motor, I_motor, V_batt, y[SOC],
                   out.motor_health, out.batt_health, out.overall_health, out.alert,
                   out.rth_threshold, out.soc_needed, out.mission_state)
        delay = out.speed_cmd
        # 6. integrate to the next tick
        if k < n - 1:
            y = plant.step_ode3(t, y, v_cmd, h)

    return SimResult(t_log, data, p)
