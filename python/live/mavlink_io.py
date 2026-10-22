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
