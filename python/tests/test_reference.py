"""The Python port must reproduce the MATLAB/Simulink reference results.

These are the numbers in CLAUDE.md under "Reference results - must not drift",
each produced by the Simulink model. Run from the python/ folder:

    python -m unittest -v
"""

import unittest
from dataclasses import replace

from droneguard import SimParams, simulate, summarize
from droneguard.summary import ABORT_ENERGY, ABORT_HEALTH, COMPLETE


def run(**changes):
    r = simulate(replace(SimParams(), **changes))
    return r, summarize(r)


class ReferenceResults(unittest.TestCase):

    def assertRounds(self, value, expected, msg=""):
        """MATLAB prints these with %.0f, so compare at that precision."""
        self.assertIsNotNone(value, msg)
        self.assertEqual(round(value), expected, msg)

    def test_default_dynamic_rth(self):
        _, s = run()
        self.assertEqual(s.turn_reason, ABORT_ENERGY)
        self.assertRounds(s.turn_t, 321, "turn time")
        self.assertRounds(s.turn_dist, 3817, "turn distance")
        self.assertRounds(s.home_t, 649, "home time")
        self.assertRounds(100 * s.home_soc, 10, "charge at home")
        self.assertRounds(s.warn_t, 529, "WARN time")
        self.assertRounds(s.alarm_t, 613, "ALARM time")

    def test_fixed_rule_runs_out(self):
        _, s = run(Use_Dynamic_RTH=0)
        self.assertEqual(s.turn_reason, COMPLETE)
        self.assertRounds(s.turn_dist, 6001, "waypoint distance")
        self.assertRounds(s.turn_t, 503, "waypoint time")
        self.assertIsNone(s.home_t, "should not make it home")
        self.assertRounds(s.dead_t, 687, "battery dead time")
        self.assertRounds(s.dead_dist, 3913, "distance from home when dead")

    def test_severe_motor_fault_health_abort(self):
        _, s = run(MotorFault_Start=30, MotorFault_Slope=1 / 200, k_eff_fault=0.80)
        self.assertEqual(s.turn_reason, ABORT_HEALTH)
        self.assertRounds(s.turn_t, 145, "abort time")
        self.assertRounds(s.turn_dist, 1710, "abort distance")
        self.assertRounds(100 * s.turn_soc, 81, "charge at abort")
        self.assertRounds(s.home_t, 298, "home time")
        self.assertRounds(100 * s.home_soc, 19, "charge at home")

    def test_battery_fault_off(self):
        r, s = run(BattFault_Mag=0)
        self.assertRounds(s.end_overall, 56, "overall health at mission end")
        self.assertIsNone(s.alarm_t, "ALARM should never fire")


class ModuleBehaviour(unittest.TestCase):

    def test_health_is_min_not_mean(self):
        from droneguard import Thresholds, health
        c = Thresholds()
        # Hot motor (90 C -> 25), healthy battery: overall must follow the motor.
        h = health(90.0, 20.0, c.V_empty + (c.V_full - c.V_empty) * 0.8 - 20 * c.R_int,
                   0.8, c)
        self.assertAlmostEqual(h.motor, 25.0)
        self.assertAlmostEqual(h.battery, 100.0)
        self.assertAlmostEqual(h.overall, 25.0)
        self.assertEqual(h.alert, 2)

    def test_two_modules_do_not_share_memory(self):
        # MATLAB's persistent variables are global; the Python objects must not be.
        from droneguard import DroneGuardModule
        a, b = DroneGuardModule(), DroneGuardModule()
        a.step(30, 18, 16.4, 1.0, 7000)          # past the waypoint -> a turns back
        out = b.step(30, 18, 16.4, 1.0, 100)
        self.assertEqual(out.mission_state, 0)


if __name__ == "__main__":
    unittest.main()
