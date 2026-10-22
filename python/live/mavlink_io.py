"""The telemetry DroneGuard needs, as real MAVLink messages - both directions.

A drone (virtual, SITL or a real Pixhawk) sends, every tick:

    ESC_TELEMETRY_1_TO_4   per-motor temperature [degC] and current [cA]
    SYS_STATUS             pack voltage [mV] and current [cA]
    BATTERY_STATUS         charge used so far [mAh]
    LOCAL_POSITION_NED     position from home [m] and velocity - sent LAST,
                           so its arrival marks a complete frame

and DroneGuard sends back one thing: COMMAND_LONG NAV_RETURN_TO_LAUNCH.

The ArduPilot dialect is used because it has the ESC telemetry message; it is
a superset of the common one, so PX4 traffic parses too.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pymavlink.dialects.v20 import ardupilotmega as mav

MAV_CMD_NAV_RETURN_TO_LAUNCH = mav.MAV_CMD_NAV_RETURN_TO_LAUNCH
DRONE_SYSTEM, DRONEGUARD_SYSTEM = 1, 42
UINT16_MAX = 65535


@dataclass
class Frame:
    """One tick of telemetry, in the units DroneGuard works in."""
    t: float                    # [s] since boot
    T_motor: float | None       # [degC] hottest motor, None if no ESC telemetry
    I_motor: float              # [A] total current to the motors
    V_batt: float               # [V] pack voltage
    soc: float                  # [-] 0..1, from charge used and capacity
    dist_home: float            # [m] horizontal distance from home
    alt: float                  # [m]
    speed: float                # [m/s] along track, negative = towards home


def link(system: int) -> mav.MAVLink:
    """A MAVLink encoder/decoder not tied to any socket (pack and parse bytes)."""
    m = mav.MAVLink(None, srcSystem=system, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1
                    if system == DRONE_SYSTEM else mav.MAV_COMP_ID_ONBOARD_COMPUTER)
    m.robust_parsing = True
    return m


def encode_frame(f: Frame, capacity_Ah: float, n_motors: int = 4) -> list:
    """Frame -> the four messages a flight controller would send."""
    ms = int(round(f.t * 1000))
    per_motor = f.I_motor / n_motors
    temps = [0 if f.T_motor is None else max(0, min(255, round(f.T_motor)))] * 4
    cur_cA = [max(0, min(UINT16_MAX, round(per_motor * 100)))] * 4
    volt_cV = [max(0, min(UINT16_MAX, round(f.V_batt * 100)))] * 4
    used_mAh = round((1.0 - f.soc) * capacity_Ah * 1000)
    I_cA = max(0, min(32767, round(f.I_motor * 100)))
    V_mV = max(0, min(UINT16_MAX - 1, round(f.V_batt * 1000)))
    msgs = []
    if f.T_motor is not None:
        msgs.append(mav.MAVLink_esc_telemetry_1_to_4_message(
            temps, volt_cV, cur_cA, [0] * 4, [0] * 4, [0] * 4))
    msgs += [
        mav.MAVLink_sys_status_message(0, 0, 0, 0, V_mV, I_cA,
                                       max(0, min(100, round(100 * f.soc))), 0, 0, 0, 0, 0, 0),
        mav.MAVLink_battery_status_message(
            0, mav.MAV_BATTERY_FUNCTION_ALL, mav.MAV_BATTERY_TYPE_LIPO, 32767,
            [UINT16_MAX] * 10, I_cA, used_mAh, -1, max(0, min(100, round(100 * f.soc)))),
        mav.MAVLink_local_position_ned_message(ms, f.dist_home, 0.0, -f.alt, f.speed, 0.0, 0.0),
    ]
    return msgs


def rtl_command(target_system: int = DRONE_SYSTEM) -> mav.MAVLink_command_long_message:
    return mav.MAVLink_command_long_message(
        target_system, mav.MAV_COMP_ID_AUTOPILOT1, MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0)


class FrameDecoder:
    """Collects messages as they arrive; returns a Frame when one is complete.

    Works with any MAVLink source: fields missing from the stream (SITL has no
    ESC telemetry) fall back - current from the battery, temperature unknown."""

    def __init__(self, capacity_Ah: float, min_dt: float = 0.095):
        self.capacity_Ah = capacity_Ah
        self.min_dt = min_dt
        self.T_motor = None
        self.I_esc = None
        self.I_batt = 0.0
        self.V_batt = 0.0
        self.used_mAh = None
        self.remaining = None
        self.last_t = None

    def feed(self, msg) -> Frame | None:
        kind = msg.get_type()
        if kind == "ESC_TELEMETRY_1_TO_4":
            # The hottest motor is the one that matters; current is the sum.
            self.T_motor = float(max(msg.temperature))
            self.I_esc = sum(msg.current) / 100.0
        elif kind == "SYS_STATUS":
            if msg.voltage_battery not in (0, UINT16_MAX):
                self.V_batt = msg.voltage_battery / 1000.0
            if msg.current_battery >= 0:
                self.I_batt = msg.current_battery / 100.0
            if msg.battery_remaining >= 0:
                self.remaining = msg.battery_remaining / 100.0
        elif kind == "BATTERY_STATUS":
            if msg.current_consumed >= 0:
                self.used_mAh = msg.current_consumed
        elif kind == "LOCAL_POSITION_NED":
            t = msg.time_boot_ms / 1000.0
            if self.last_t is not None and t - self.last_t < self.min_dt:
                return None
            self.last_t = t
            if self.used_mAh is not None:
                soc = 1.0 - self.used_mAh / (self.capacity_Ah * 1000.0)
            else:
                soc = self.remaining if self.remaining is not None else 1.0
            return Frame(
                t=t,
                T_motor=self.T_motor,
                I_motor=self.I_esc if self.I_esc is not None else self.I_batt,
                V_batt=self.V_batt,
                soc=min(max(soc, 0.0), 1.0),
                dist_home=math.hypot(msg.x, msg.y),
                alt=-msg.z,
                speed=msg.vx if abs(msg.vx) >= abs(msg.vy) else msg.vy,
            )
        return None


# ---------------------------------------------------------------------------
# Remote control of a virtual drone running on another machine (e.g. the Pi).
# These use MAVLink's USER commands: simulation-only, a real autopilot ignores them.

CMD_INJECT_FAULT = mav.MAV_CMD_USER_1     # param1: 1 = motor, 2 = battery
CMD_START_FLIGHT = mav.MAV_CMD_USER_2     # see start_command()
CMD_STOP_FLIGHT = mav.MAV_CMD_USER_3
FAULT_CODES = {"motor": 1, "battery": 2}

# ArduCopter flight modes, as the real autopilot reports them in HEARTBEAT.
MODE_IDLE, MODE_AUTO, MODE_RTL, MODE_LAND = (mav.COPTER_MODE_STABILIZE, mav.COPTER_MODE_AUTO,
                                             mav.COPTER_MODE_RTL, mav.COPTER_MODE_LAND)
MODE_NAMES = {MODE_IDLE: "STABILIZE (on the ground)", MODE_AUTO: "AUTO (mission)",
              MODE_RTL: "RTL (returning home)", MODE_LAND: "LAND (landed)"}
FRAME_TYPE = {4: mav.MAV_TYPE_QUADROTOR, 6: mav.MAV_TYPE_HEXAROTOR, 8: mav.MAV_TYPE_OCTOROTOR}


def _command(cmd: int, *params: float) -> mav.MAVLink_command_long_message:
    p = list(params) + [0.0] * (7 - len(params))
    return mav.MAVLink_command_long_message(DRONE_SYSTEM, mav.MAV_COMP_ID_AUTOPILOT1, cmd, 0, *p)


def start_command(plan: dict | None, faults: bool, speed: float) -> mav.MAVLink_command_long_message:
    """Pack a whole flight request into one COMMAND_LONG (7 numbers):
    speed, faults + 10*motors, payload_kg (0 = reference drone), endurance_min,
    distance_km, cruise_mps, altitude_m."""
    plan = plan or {}
    motors = int(plan.get("motors") or 0)
    return _command(CMD_START_FLIGHT, speed, float(faults) + 10 * motors,
                    plan.get("payload_kg", 0.0), plan.get("endurance_min", 0.0),
                    plan.get("distance_km", 0.0), plan.get("cruise_mps", 0.0),
                    plan.get("altitude_m", 0.0))


def parse_start(msg) -> tuple[dict | None, bool, float]:
    """The other half of start_command(): (plan or None, faults, speed)."""
    faults = int(round(msg.param2)) % 10 == 1
    motors = int(round(msg.param2)) // 10
    if msg.param3 <= 0:
        return None, faults, msg.param1
    plan = {"payload_kg": msg.param3, "endurance_min": msg.param4, "distance_km": msg.param5,
            "cruise_mps": msg.param6, "altitude_m": msg.param7, "motors": motors or None}
    return plan, faults, msg.param1


def fault_command(kind: str) -> mav.MAVLink_command_long_message:
    return _command(CMD_INJECT_FAULT, FAULT_CODES[kind])


def stop_command() -> mav.MAVLink_command_long_message:
    return _command(CMD_STOP_FLIGHT)


def heartbeat(mode: int, flying: bool, n_motors: int = 4) -> mav.MAVLink_heartbeat_message:
    base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if flying else 0)
    return mav.MAVLink_heartbeat_message(
        FRAME_TYPE.get(n_motors, mav.MAV_TYPE_QUADROTOR), mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base, mode, mav.MAV_STATE_ACTIVE if flying else mav.MAV_STATE_STANDBY, 3)


def truth_messages(t: float, faults: dict) -> list:
    """Simulation-only ground truth, so a dashboard can show what is really wrong."""
    ms = int(round(t * 1000))
    return [mav.MAVLink_named_value_float_message(ms, b"f_motor", faults["motor"]),
            mav.MAVLink_named_value_float_message(ms, b"f_batt", faults["battery"])]


def status_text(text: str, severity: int = mav.MAV_SEVERITY_INFO) -> mav.MAVLink_statustext_message:
    return mav.MAVLink_statustext_message(severity, text[:50].encode())
