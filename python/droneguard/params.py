"""Aircraft and scenario numbers for the simulation.

Mirrors matlab/sim/droneguard_params.m. Change a value with
dataclasses.replace(SimParams(), k_eff_fault=0.8) rather than editing defaults.
"""

from dataclasses import dataclass

from .thresholds import Thresholds

_C = Thresholds()


@dataclass(frozen=True)
class SimParams:
    # ---- solver / mission ----
    Ts: float = 0.1             # [s]   module rate and solver step
    T_end: float = 800.0        # [s]   how long we let the mission run
    Alt_cmd: float = 50.0       # [m]   commanded cruise altitude
    Use_Dynamic_RTH: float = 1  # 1 = health-aware return-to-home, 0 = fixed 25%

    # ---- airframe ----
    m_drone: float = 2.0        # [kg]
    g_accel: float = 9.81       # [m/s^2]
    Cd_forward: float = 0.55    # [N/(m/s)] linear forward drag
    Accel_max: float = 2.5      # [m/s^2] slew limit on the speed command
    Speed_init: float = 0.0     # [m/s]
    Alt_init: float | None = None   # [m] None = start at Alt_cmd

    # ---- flight controller gains ----
    Kp_alt: float = 8.0
    Ki_alt: float = 0.5
    Kd_alt: float = 12.0
    N_alt: float = 100.0        # derivative filter coefficient (Simulink PID default)
    Kp_spd: float = 6.0
    Ki_spd: float = 2.0

    # ---- propulsion ----
    k_thrust: float = 1.09      # [N/A]  thrust per amp when healthy
    K_therm: float = 0.055      # [degC/A^2]
    tau_therm: float = 40.0     # [s]
    T_ambient: float = 30.0     # [degC]

    # ---- battery (pack constants shared with the health algorithm) ----
    V_full: float = _C.V_full
    V_empty: float = _C.V_empty
    R_int: float = _C.R_int
    Cap_Ah: float = _C.Cap_Ah
    SOC_init: float = 1.0

    # ---- fault 1: motor / ESC degradation ----
    MotorFault_Start: float = 150.0
    MotorFault_Mag: float = 1.0
    k_eff_fault: float = 0.50
    MotorFault_Slope: float | None = None   # None = Mag / (T_end - Start)

    # ---- fault 2: battery ageing ----
    BattFault_Start: float = 400.0
    BattFault_Mag: float = 1.0
    k_R_fault: float = 3.00
    k_drain_fault: float = 0.67
    BattFault_Slope: float | None = None

    @property
    def alt_init(self) -> float:
        return self.Alt_cmd if self.Alt_init is None else self.Alt_init

    @property
    def motor_slope(self) -> float:
        if self.MotorFault_Slope is not None:
            return self.MotorFault_Slope
        return self.MotorFault_Mag / (self.T_end - self.MotorFault_Start)

    @property
    def batt_slope(self) -> float:
        if self.BattFault_Slope is not None:
            return self.BattFault_Slope
        return self.BattFault_Mag / (self.T_end - self.BattFault_Start)
