"""Parts the sizing engine can choose from.

Seeded from Table 2 of Vishwath et al. (2022), "A guide to novice for proper
selection of the components of drone for specific applications" - the
`7.Component selection.pdf` in the repo root. Each row is a motor + propeller
combination measured on a thrust rig: the current it drew and the thrust it
made at full throttle, at the stated voltage.

The paper gives no masses, so motor, propeller, ESC, frame and battery masses
are ESTIMATES from simple rules of thumb, marked as such. Replace them with
datasheet numbers when real parts are picked.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

CELL_V_NOMINAL = 3.7     # [V]  LiPo nominal voltage per cell
CELL_V_FULL = 4.2        # [V]
CELL_V_EMPTY = 3.5       # [V]  under load; the paper's 3.0 V is the damage limit

LIPO_WH_PER_KG = 150.0   # pack-level energy density, typical hobby LiPo  (estimate)
MOTOR_G_PER_CM3 = 3.2    # motor mass per cm^3 of motor can volume         (estimate)
PROP_G_PER_IN2 = 0.15    # propeller mass per (diameter in inches)^2       (estimate)
ESC_G_PER_A = 0.9        # ESC mass per amp of rating, plus ESC_G_BASE     (estimate)
ESC_G_BASE = 5.0
FRAME_G_BASE = 100.0     # centre plates, landing gear                     (estimate)
FRAME_G_PER_ARM_IN = 8.0 # each arm, per inch of propeller it must clear   (estimate)

ESC_RATINGS_A = (12, 20, 30, 40, 50, 60, 70, 80, 100, 120, 150, 200)
BATTERY_C_RATINGS = (15, 20, 25, 30, 35, 45, 50, 60, 75, 100)


@dataclass(frozen=True)
class MotorProp:
    """One measured motor + propeller combination (one row of Table 2)."""
    size_mm: tuple[float, float]   # the two motor dimensions as printed (L*B)
    kv: float                      # [rpm/V]
    prop: str                      # "diameter*pitch" in inches
    I_max: float                   # [A]  current at full throttle on the rig
    V_test: float                  # [V]  rig voltage
    T_max_g: float                 # [g]  thrust at full throttle

    @property
    def name(self) -> str:
        return f"{self.size_mm[0]:g}x{self.size_mm[1]:g} mm {self.kv:g}kV + {self.prop}\" prop"

    @property
    def prop_diameter_in(self) -> float:
        return float(self.prop.split("*")[0])

    @property
    def cells(self) -> int:
        """LiPo cell count that matches the rig voltage."""
        return max(1, round(self.V_test / CELL_V_NOMINAL))

    @property
    def P_max(self) -> float:
        """[W] electrical power at full throttle."""
        return self.V_test * self.I_max

    @property
    def motor_g(self) -> float:
        """Estimated motor mass: the larger dimension is taken as the diameter."""
        d, h = max(self.size_mm), min(self.size_mm)
        volume_cm3 = math.pi * (d / 20) ** 2 * (h / 10)
        return MOTOR_G_PER_CM3 * volume_cm3

    @property
    def prop_g(self) -> float:
        return PROP_G_PER_IN2 * self.prop_diameter_in ** 2

    def current_at(self, thrust_g: float) -> float:
        """[A] current to make `thrust_g`. Propeller momentum theory gives
        power ~ thrust^1.5 at fixed voltage, anchored at the measured full-throttle point."""
        return self.I_max * (thrust_g / self.T_max_g) ** 1.5


# Table 2, in the paper's order. Rows 6 and 7 (2333 g and 2992 g) list the same
# motor and propeller with different thrust - copied as published.
TABLE_2: tuple[MotorProp, ...] = (
    MotorProp((19.1, 28.9), 2550, "6*4", 32.9, 16.2, 1406),
    MotorProp((26.4, 41.8), 580, "14*4.8", 16.0, 15.0, 1500),
    MotorProp((48.0, 35.3), 790, "13*6.5", 29.4, 12.0, 1755),
    MotorProp((36.0, 35.3), 1450, "11*7", 51.5, 11.8, 1884),
    MotorProp((48.0, 35.5), 1150, "9*6", 54.6, 11.4, 2241),
    MotorProp((26.0, 34.0), 925, "9*5", 50.5, 25.0, 2333.333333),
    MotorProp((26.0, 34.4), 925, "9*5", 50.5, 24.5, 2992),
    MotorProp((34.0, 49.0), 900, "13*6.5", 57.8, 13.3, 3271),
    MotorProp((45.0, 42.8), 800, "12*6", 58.6, 20.7, 3635),
    MotorProp((26.5, 86.8), 135, "30*10.5", 21.0, 24.0, 4333.333333),
    MotorProp((45.0, 60.7), 490, "18*6.1", 62.0, 25.0, 5600),
    MotorProp((45.0, 60.7), 490, "18*6.1", 62.0, 25.0, 5750),
    MotorProp((33.5, 69.0), 340, "22*6.6", 57.8, 24.0, 6000),
    MotorProp((26.1, 87.1), 190, "29*9.5", 44.0, 24.0, 7250),
    MotorProp((46.3, 60.0), 220, "19*10", 60.0, 44.0, 8750),
    MotorProp((36.0, 57.6), 490, "16*5.4", 83.2, 33.0, 9259),
)


def esc_rating(I_motor_max: float) -> int:
    """Smallest standard ESC that keeps the motor's peak at or below 80% of rating."""
    need = I_motor_max / 0.8
    for r in ESC_RATINGS_A:
        if r >= need:
            return r
    return math.ceil(need)


def esc_g(rating_A: float) -> float:
    return ESC_G_BASE + ESC_G_PER_A * rating_A


def frame_g(n_motors: int, prop_diameter_in: float) -> float:
    return FRAME_G_BASE + n_motors * FRAME_G_PER_ARM_IN * prop_diameter_in


def battery_g(cells: int, capacity_Ah: float) -> float:
    return cells * CELL_V_NOMINAL * capacity_Ah / LIPO_WH_PER_KG * 1000.0
