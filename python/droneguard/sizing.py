"""Sizing engine: from "what must it carry, for how long" to a parts list.

The method is the paper's (Vishwath et al. 2022), with its gaps closed:

    1. guess the all-up weight (AUW)       = payload + avionics + frame
    2. thrust each motor must reach        = thrust_ratio x AUW / motors   (2:1 rule)
    3. pick a motor + propeller from the catalogue that reaches it
    4. ESC so the motor's peak current is at most 80% of its rating
    5. battery: cells from the motor's voltage; capacity so hovering for the
       required time uses only 80% of the pack; C rating so the pack can
       deliver every motor's peak current at once
    6. add up the real masses -> new AUW, and repeat from 2 until it settles

The paper stops after one pass with a guessed AUW, which is how a battery that
cannot feed its motors got into it. Step 6 is the fix.

    python -m droneguard.sizing --payload-kg 2 --endurance-min 15
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

from . import catalogue as cat
from .catalogue import TABLE_2, MotorProp

MOTOR_COUNTS = (4, 6, 8)


@dataclass(frozen=True)
class Requirements:
    payload_g: float
    endurance_min: float = 15.0      # hover time on the usable part of the pack
    n_motors: int | None = None      # None = try 4, 6 and 8 and keep the lightest
    thrust_ratio: float = 2.0        # full-throttle thrust : weight
    usable_fraction: float = 0.8     # the 80% rule: never use more of the pack
    avionics_g: float = 300.0        # flight controller, GPS, receiver, Pi, wiring
    selection: str = "lightest"      # "lightest": try every motor, keep the lightest
                                     #   drone that works; "paper": smallest motor
                                     #   strong enough, re-picked on every pass


@dataclass
class Design:
    req: Requirements
    n_motors: int
    motor: MotorProp | None = None
    auw_g: float = 0.0
    masses_g: dict = field(default_factory=dict)
    hover_thrust_g: float = 0.0      # per motor
    I_hover: float = 0.0             # [A] per motor
    I_hover_total: float = 0.0       # [A]
    I_max_total: float = 0.0         # [A] every motor at full throttle
    esc_A: int = 0
    cells: int = 0
    capacity_Ah: float = 0.0
    c_rating: int | None = None
    endurance_min: float = 0.0
    thrust_to_weight: float = 0.0
    iterations: list = field(default_factory=list)   # AUW after each pass
    converged: bool = False
    problems: list = field(default_factory=list)
    shortfall: float = 0.0     # thrust needed / thrust available, when > 1 it cannot fly

    @property
    def feasible(self) -> bool:
        return self.converged and not self.problems

    @property
    def V_nominal(self) -> float:
        return self.cells * cat.CELL_V_NOMINAL

    @property
    def hover_power_W(self) -> float:
        return self.I_hover_total * self.V_nominal

    def text(self) -> str:
        r = self.req
        lines = [f"--- DroneGuard sizing: {r.payload_g / 1000:g} kg payload, "
                 f"{r.endurance_min:g} min hover, {self.n_motors} motors ---"]
        if not self.converged:
            lines.append("NOT POSSIBLE with the parts in the catalogue")
            lines += [f"  - {p}" for p in self.problems]
            return "\n".join(lines)
        m = self.motor
        lines += [
            f"All-up weight       : {self.auw_g / 1000:.2f} kg  "
            f"(settled after {len(self.iterations)} passes)",
            f"Motor + propeller   : {m.name}, {m.T_max_g:.0f} g max thrust each",
            f"ESC                 : {self.esc_A} A each  (motor peak {m.I_max:.1f} A)",
            f"Battery             : {self.cells}S {self.capacity_Ah * 1000:.0f} mAh "
            + (f"{self.c_rating}C" if self.c_rating else "(no standard C rating is enough)"),
            f"Thrust : weight     : {self.thrust_to_weight:.2f} : 1",
            f"Hover               : {self.hover_thrust_g:.0f} g per motor, "
            f"{self.I_hover_total:.1f} A total, {self.hover_power_W:.0f} W",
            f"Full throttle       : {self.I_max_total:.0f} A total, "
            f"{self.I_max_total * self.V_nominal:.0f} W",
            f"Hover endurance     : {self.endurance_min:.1f} min on "
            f"{100 * r.usable_fraction:.0f}% of the pack",
            "Mass estimate [g]   : " + ", ".join(f"{k} {v:.0f}" for k, v in self.masses_g.items()),
        ]
        lines.append("Verdict             : " + ("OK" if self.feasible else
                                                "PROBLEM - " + "; ".join(self.problems)))
        return "\n".join(lines)


def _pass(req: Requirements, n: int, auw_g: float, d: Design,
          only: MotorProp | None) -> float | None:
    """One trip round steps 2-6. Fills `d` and returns the new AUW, or None."""
    need_g = req.thrust_ratio * auw_g / n
    pool = TABLE_2 if only is None else (only,)
    candidates = [m for m in pool if m.T_max_g >= need_g]
    if not candidates:
        strongest = max(m.T_max_g for m in pool)
        d.shortfall = need_g / strongest
        d.problems = [f"each motor must make {need_g:.0f} g at {auw_g / 1000:.1f} kg "
                      f"all-up weight; the strongest available makes {strongest:.0f} g"]
        return None
    hover_g = auw_g / n
    m = min(candidates, key=lambda m: m.T_max_g)   # smallest that is strong enough

    I_hover = m.current_at(hover_g)
    I_hover_total = n * I_hover
    capacity = I_hover_total * (req.endurance_min / 60) / req.usable_fraction
    capacity = math.ceil(capacity * 10) / 10          # round up to 100 mAh
    I_max_total = n * m.I_max
    c_rating = next((c for c in cat.BATTERY_C_RATINGS if c * capacity >= I_max_total), None)
    esc = cat.esc_rating(m.I_max)

    masses = {
        "payload": req.payload_g,
        "avionics": req.avionics_g,
        "frame": cat.frame_g(n, m.prop_diameter_in),
        "motors": n * m.motor_g,
        "props": n * m.prop_g,
        "ESCs": n * cat.esc_g(esc),
        "battery": cat.battery_g(m.cells, capacity),
    }
    d.motor, d.hover_thrust_g, d.I_hover, d.I_hover_total = m, hover_g, I_hover, I_hover_total
    d.I_max_total, d.esc_A, d.cells, d.capacity_Ah = I_max_total, esc, m.cells, capacity
    d.c_rating, d.masses_g = c_rating, masses
    return sum(masses.values())


def size_fixed(req: Requirements, n: int, motor: MotorProp | None = None,
               max_passes: int = 100, max_auw_g: float = 150_000) -> Design:
    """Size for one motor count, either with one given motor or re-picking the
    smallest strong-enough motor on every pass (the paper's rule)."""
    d = Design(req, n)
    auw = req.payload_g + req.avionics_g + cat.frame_g(n, 10)     # step 1
    for _ in range(max_passes):
        new = _pass(req, n, auw, d, motor)
        if new is None:
            d.auw_g = auw
            return d
        d.iterations.append(new)
        if new > max_auw_g:
            d.auw_g = new
            d.shortfall = math.inf
            d.problems = ["weight runs away: each extra gram of battery needs more "
                          "battery to lift it - ask for less endurance or payload"]
            return d
        if abs(new - auw) < 0.5:
            d.converged = True
            auw = new
            break
        auw = new
    else:
        d.problems = ["did not settle (the motor choice keeps flipping)"]

    # Final numbers at the settled weight.
    d.auw_g = auw
    d.hover_thrust_g = auw / n
    d.I_hover = d.motor.current_at(d.hover_thrust_g)
    d.I_hover_total = n * d.I_hover
    d.endurance_min = req.usable_fraction * d.capacity_Ah / d.I_hover_total * 60
    d.thrust_to_weight = n * d.motor.T_max_g / auw

    # Checks. Steps 2-5 should guarantee these; they are here so a hand-edited
    # design (or the paper's) is held to the same rules.
    if d.thrust_to_weight < req.thrust_ratio - 1e-9:
        d.problems.append(f"thrust:weight {d.thrust_to_weight:.2f} below {req.thrust_ratio:g}")
    if d.c_rating is None:
        d.problems.append(f"no {max(cat.BATTERY_C_RATINGS)}C-or-lower {d.capacity_Ah:.1f} Ah "
                          f"pack can deliver {d.I_max_total:.0f} A")
    if d.endurance_min < req.endurance_min - 0.05:
        d.problems.append(f"hovers {d.endurance_min:.1f} min, wanted {req.endurance_min:g}")
    return d


def candidates(req: Requirements) -> list[Design]:
    """Every design worth considering for these requirements."""
    counts = MOTOR_COUNTS if req.n_motors is None else (req.n_motors,)
    if req.selection == "paper":
        return [size_fixed(req, n) for n in counts]
    return [size_fixed(req, n, m) for n in counts for m in TABLE_2]


def size(req: Requirements) -> Design:
    """The lightest design that passes every check. If none does, the one that
    came closest, so its problems say what to change."""
    designs = candidates(req)
    good = [d for d in designs if d.feasible]
    if good:
        return min(good, key=lambda d: d.auw_g)

    # Nothing works. Return the closest miss and say what would fix it.
    built = [d for d in designs if d.converged]
    if built:       # it flies, but fails a check (battery C rating, endurance)
        return min(built, key=lambda d: (len(d.problems), d.auw_g))
    best = min(designs, key=lambda d: d.shortfall)
    tried = f"all {len(designs)} motor/layout combinations were tried"
    if math.isinf(best.shortfall):
        best.problems = [best.problems[0], tried]
    else:
        best.problems = [
            best.problems[0],
            f"{tried}; the closest needs at least {100 * (best.shortfall - 1):.1f}% "
            "more thrust per motor (more, once the bigger motors' weight is added)",
            "fix: add stronger motors to the catalogue, use more motors, "
            "or ask for less payload or endurance",
        ]
    return best


# --------------------------------------------------------------------------
# Battery check on its own, for judging a pack someone already chose.

def battery_can_feed(capacity_Ah: float, c_rating: float, n_motors: int,
                     I_motor_max: float) -> tuple[bool, float, float]:
    """(ok, amps the pack can deliver, amps the motors can draw at full throttle)."""
    deliverable = capacity_Ah * c_rating
    needed = n_motors * I_motor_max
    return deliverable >= needed, deliverable, needed


# --------------------------------------------------------------------------
# Cost: the paper's Table 4 (2022 INR, basic flight parts only), interpolated
# by payload. It is the paper's price for its own builds, not for our parts.

_TABLE_4_PAYLOAD = (2000, 5000, 10000, 15000, 20000, 25000, 30000)
_TABLE_4 = {
    "motors": (17600, 28800, 35000, 42000, 56000, 56000, 72000),
    "ESCs": (4000, 12000, 17000, 24000, 36000, 32000, 36000),
    "props": (1800, 3000, 4200, 5400, 9600, 9600, 12000),
    "battery": (5000, 8000, 28000, 40000, 50000, 50000, 75000),
    "flight controller": (20000, 33800, 33800, 33800, 56400, 56400, 26400),
    "frame": (5900, 10700, 10700, 10700, 49250, 49250, 49250),
}


def cost_inr_2022(payload_g: float) -> dict:
    x = min(max(payload_g, _TABLE_4_PAYLOAD[0]), _TABLE_4_PAYLOAD[-1])
    for i in range(len(_TABLE_4_PAYLOAD) - 1):
        x0, x1 = _TABLE_4_PAYLOAD[i], _TABLE_4_PAYLOAD[i + 1]
        if x <= x1:
            w = (x - x0) / (x1 - x0)
            break
    out = {k: v[i] + w * (v[i + 1] - v[i]) for k, v in _TABLE_4.items()}
    out["total"] = sum(out.values())
    return out


# --------------------------------------------------------------------------
# The paper's seven builds (Table 3), for checking ourselves against it.
# "Total require thrust" in Table 3 is really the paper's AUW guess: its
# single-motor thrust column is exactly 2 x that / motors.

@dataclass(frozen=True)
class PaperCase:
    payload_g: float
    auw_guess_g: float
    n_motors: int
    motor_index: int          # row of TABLE_2 the paper chose
    claimed_power_W: float    # Table 3 "Power Required"


PAPER_CASES = (
    PaperCase(2000, 3000, 4, 1, 46517.6),
    PaperCase(5000, 7000, 6, 5, 114211.80),
    PaperCase(10000, 13000, 6, 9, 162407.79),
    PaperCase(15000, 18000, 6, 12, 136794.24),
    PaperCase(20000, 23000, 8, 11, 161893.10),
    PaperCase(25000, 29000, 8, 13, 182956.3),
    PaperCase(30000, 35000, 8, 14, 358563.33),
)


def paper_pick(case: PaperCase, thrust_ratio: float = 2.0) -> MotorProp:
    """The paper's one-pass rule: smallest catalogue thrust >= 2 x AUW guess / motors."""
    need = thrust_ratio * case.auw_guess_g / case.n_motors
    return min((m for m in TABLE_2 if m.T_max_g >= need - 1e-6), key=lambda m: m.T_max_g)


# The paper's firefighting drone: eight motors on a 6S 1200 mAh 15C pack.
FIREFIGHTING_PACK = dict(cells=6, capacity_Ah=1.2, c_rating=15, n_motors=8)


def print_paper_cases(endurance_min: float = 15.0) -> None:
    print("The paper's seven builds (Table 3) against this engine, "
          f"{endurance_min:g} min hover, same motor counts\n")
    print(f"{'payload':>8} {'motors':>6} | {'paper: motor':>12} {'AUW guess':>9} "
          f"{'claimed W':>10} {'real max W':>10} | {'ours: AUW':>9} {'motor':>7} "
          f"{'battery':>17}  verdict")
    for c in PAPER_CASES:
        pm = TABLE_2[c.motor_index]
        d = size(Requirements(c.payload_g, endurance_min, c.n_motors))
        head = (f"{c.payload_g / 1000:7g}k {c.n_motors:>6} | {pm.T_max_g:11.0f}g "
                f"{c.auw_guess_g / 1000:8.0f}k {c.claimed_power_W:10.0f} "
                f"{c.n_motors * pm.P_max:10.0f} | ")
        if d.converged:
            tail = (f"{d.auw_g / 1000:8.1f}k {d.motor.T_max_g:6.0f}g "
                    f"{d.cells:>3}S {d.capacity_Ah * 1000:6.0f}mAh {d.c_rating or '-':>3}C  "
                    + ("OK" if d.feasible else "PROBLEM"))
        else:
            tail = (f"{'-':>9} {'-':>7} {'-':>17}  NOT POSSIBLE "
                    f"(at least {100 * (d.shortfall - 1):.1f}% short on thrust)")
        print(head + tail)

    fp = FIREFIGHTING_PACK
    weakest = min(TABLE_2, key=lambda m: m.I_max)
    ok, have, need = battery_can_feed(fp["capacity_Ah"], fp["c_rating"], fp["n_motors"],
                                      weakest.I_max)
    print(f"\nFirefighting pack ({fp['cells']}S {fp['capacity_Ah'] * 1000:.0f} mAh "
          f"{fp['c_rating']}C, {fp['n_motors']} motors): delivers {have:.0f} A; even the "
          f"lowest-current motor in the catalogue needs {need:.0f} A -> "
          + ("OK" if ok else "CANNOT FEED ITS MOTORS"))


def main():
    ap = argparse.ArgumentParser(prog="python -m droneguard.sizing")
    ap.add_argument("--payload-kg", type=float, default=2.0)
    ap.add_argument("--endurance-min", type=float, default=15.0)
    ap.add_argument("--motors", type=int, choices=MOTOR_COUNTS)
    ap.add_argument("--selection", choices=["lightest", "paper"], default="lightest")
    ap.add_argument("--paper-cases", action="store_true",
                    help="size the paper's seven payloads and compare")
    a = ap.parse_args()

    if a.paper_cases:
        print_paper_cases(a.endurance_min)
        return

    req = Requirements(a.payload_kg * 1000, a.endurance_min, a.motors, selection=a.selection)
    d = size(req)
    print(d.text())
    cost = cost_inr_2022(req.payload_g)
    print(f"Cost (paper Table 4): Rs {cost['total']:,.0f}  (2022 prices, basic flight parts only)")


if __name__ == "__main__":
    main()
