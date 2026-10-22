"""The design check (P2): sized drone -> simulation -> GO / NO-GO.

    python -m unittest -v
"""

import unittest
from dataclasses import fields

from droneguard import SimParams, Thresholds, simulate, summarize
from droneguard.check import Mission, check, to_sim
from droneguard.sizing import Design, Requirements, size
from droneguard.summary import ABORT_ENERGY


def reference_design() -> Design:
    """A 'design' describing the reference drone: 2 kg, 18 A hover, 4S 5 Ah."""
    d = Design(Requirements(0), n_motors=4)
    d.auw_g, d.I_hover_total, d.cells, d.capacity_Ah = 2000, 18.0, 4, 5.0
    d.converged = True
    return d


class MappingIsAnchored(unittest.TestCase):

    def test_reference_design_maps_to_reference_parameters(self):
        p, c = to_sim(reference_design(), Mission(6000), faults=True, T_end=800, fault_by=800)
        ref_p = SimParams()
        for name, mine, ref in [(f.name, getattr(p, f.name), getattr(ref_p, f.name))
                                for f in fields(SimParams)]:
            if ref is None:      # slopes: compared through the properties below
                continue
            with self.subTest(param=name):
                self.assertAlmostEqual(mine, ref, places=9)
        self.assertAlmostEqual(p.motor_slope, ref_p.motor_slope, places=12)
        self.assertAlmostEqual(p.batt_slope, ref_p.batt_slope, places=12)
        for f in fields(Thresholds):
            with self.subTest(threshold=f.name):
                self.assertAlmostEqual(getattr(c, f.name), getattr(Thresholds(), f.name), places=9)

    def test_reference_design_flies_the_reference_mission(self):
        p, c = to_sim(reference_design(), Mission(6000), faults=True, T_end=800, fault_by=800)
        s = summarize(simulate(p, c), c)
        self.assertEqual(s.turn_reason, ABORT_ENERGY)
        self.assertEqual(round(s.turn_t), 321)
        self.assertEqual(round(s.turn_dist), 3817)
        self.assertEqual(round(s.home_t), 649)


class Verdicts(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.design = size(Requirements(2000, 15))

    def test_short_trip_is_go(self):
        r = check(self.design, Mission(3000))
        self.assertEqual(r.verdict, "GO", r.reasons + r.cautions)
        self.assertGreaterEqual(r.healthy.home_soc, 0.10)
        self.assertIsNotNone(r.stressed.home_t)

    def test_trip_beyond_range_is_no_go(self):
        r = check(self.design, Mission(3000))
        too_far = r.max_distance_m * 1.15
        r2 = check(self.design, Mission(too_far))
        self.assertFalse(r2.go)
        self.assertIn("cannot reach", r2.reasons[0])

    def test_long_trip_with_thin_worn_margin_is_caution(self):
        r = check(self.design, Mission(4500))
        self.assertEqual(r.verdict, "GO WITH CAUTION")
        self.assertLess(r.stressed.home_soc, 0.10)

    def test_range_estimate_agrees_with_verdicts(self):
        r = check(self.design, Mission(3000))
        self.assertTrue(check(self.design, Mission(0.9 * r.max_distance_m)).go)

    def test_unsized_design_is_refused(self):
        d = size(Requirements(6000, 45))
        with self.assertRaises(ValueError):
            to_sim(d, Mission(1000))


if __name__ == "__main__":
    unittest.main()
