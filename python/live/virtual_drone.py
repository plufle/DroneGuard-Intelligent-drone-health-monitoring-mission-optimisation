"""A drone that only exists in software but talks like a real one.

The same plant as droneguard/sim.py (flight controller loops, motors, battery,
physics), plus what a real autopilot adds on top: it flies its own mission -
out to the drop point, back, land - and obeys RETURN_TO_LAUNCH. It has no idea
DroneGuard exists; it just reports telemetry and takes commands.

Faults can be injected while it flies, to watch DroneGuard react.
"""

from dataclasses import replace

from droneguard.params import SimParams
from droneguard.sim import SOC, TM, VZ, V, X, Z, Plant
from droneguard.thresholds import Thresholds

from .mavlink_io import MAV_CMD_NAV_RETURN_TO_LAUNCH, Frame

OUTBOUND, RETURNING, LANDED = 0, 1, 2
PHASES = {OUTBOUND: "mission: outbound", RETURNING: "returning home", LANDED: "landed"}


class VirtualDrone:
    def __init__(self, p: SimParams, mission: Thresholds, n_motors: int = 4):
        self.p = p
        self.range_m = mission.Range
        self.cruise = mission.Cruise
        self.n_motors = n_motors
        self.plant = Plant(p)
        self.y = self.plant.initial_state()
        self.k = 0
        self.delay = 0.0           # same one-tick command latency as Command_Delay
        self.v_cmd = 0.0
        self.phase = OUTBOUND
        self.rtl_requested = False
        self.rtl_at = None

    # ---- what the outside world sees ------------------------------------
    @property
    def t(self) -> float:
        return self.k * self.p.Ts

    @property
    def done(self) -> bool:
        return (self.phase == LANDED or self.y[SOC] <= 0.0
                or self.t >= self.p.T_end - 1e-9)

    def telemetry(self) -> Frame:
        *_, I_motor, _, V_batt = self.plant.signals(self.t, self.y, self.v_cmd)
        return Frame(t=self.t, T_motor=self.y[TM] + self.p.T_ambient, I_motor=I_motor,
                     V_batt=V_batt, soc=self.y[SOC], dist_home=self.y[X],
                     alt=self.y[Z], speed=self.y[V])

    def command(self, msg) -> bool:
        """Handle a MAVLink command. Returns True if it was accepted."""
        if msg.get_type() == "COMMAND_LONG" and msg.command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            if not self.rtl_requested:
                self.rtl_requested, self.rtl_at = True, self.t
            return True
        return False

    # ---- one autopilot tick ---------------------------------------------
    def advance(self) -> None:
        x = self.y[X]
        if self.phase == OUTBOUND and (x >= self.range_m or self.rtl_requested):
            self.phase = RETURNING
        elif self.phase == RETURNING and x <= 1.0:
            self.phase = LANDED
        speed_cmd = {OUTBOUND: self.cruise, RETURNING: -self.cruise, LANDED: 0.0}[self.phase]

        self.delay = speed_cmd
        self.y = self.plant.step_ode3(self.t, self.y, self.v_cmd, self.p.Ts)
        self.k += 1
        step = self.p.Accel_max * self.p.Ts
        self.v_cmd += min(max(self.delay - self.v_cmd, -step), step)

    # ---- fault injection (simulation only, not MAVLink) ------------------
    def inject(self, kind: str, ramp_s: float = 200.0) -> None:
        """Start a fault now; it reaches full severity after ramp_s seconds."""
        t = self.t
        if kind == "motor":
            self.p = replace(self.p, MotorFault_Start=t, MotorFault_Mag=1.0,
                             MotorFault_Slope=1.0 / ramp_s)
        elif kind == "battery":
            self.p = replace(self.p, BattFault_Start=t, BattFault_Mag=1.0,
                             BattFault_Slope=1.0 / ramp_s)
        else:
            raise ValueError(kind)
        self.plant = Plant(self.p)

    def fault_levels(self) -> dict:
        """Ground truth - what is really wrong (DroneGuard never sees this)."""
        from droneguard.sim import _ramp
        return {"motor": _ramp(self.t, self.p.MotorFault_Start, self.plant.m_slope),
                "battery": _ramp(self.t, self.p.BattFault_Start, self.plant.b_slope)}

    @property
    def phase_name(self) -> str:
        return PHASES[self.phase]

    @property
    def vertical_rate(self) -> float:
        return self.y[VZ]
