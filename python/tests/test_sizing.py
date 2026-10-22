"""The sizing engine (P1): the paper's method, with its mistakes caught.

    python -m unittest -v
"""

import unittest

from droneguard import catalogue as cat
from droneguard.sizing import (FIREFIGHTING_PACK, PAPER_CASES, Requirements,
                               battery_can_feed, cost_inr_2022, paper_pick, size)


class PaperMethod(unittest.TestCase):

    def test_two_to_one_rule_reproduces_table_3_choices(self):
        # Single motor thrust = 2 x AUW guess / motors, then the smallest motor
        # that reaches it. This must pick exactly the paper's motor every time.
        for c in PAPER_CASES:
            with self.subTest(payload=c.payload_g):
                self.assertIs(paper_pick(c), cat.TABLE_2[c.motor_index])

    def test_table_3_power_is_impossible(self):
        # The parts can draw at most motors x volts x amps. Table 3 claims
        # 13x to 54x more than that.
        for c in PAPER_CASES:
            with self.subTest(payload=c.payload_g):
                real_max = c.n_motors * cat.TABLE_2[c.motor_index].P_max
                self.assertGreater(c.claimed_power_W / real_max, 13)
        first = PAPER_CASES[0]
        self.assertAlmostEqual(first.n_motors * cat.TABLE_2[first.motor_index].P_max, 960)

    def test_firefighting_battery_is_flagged(self):
        fp = FIREFIGHTING_PACK
        weakest = min(cat.TABLE_2, key=lambda m: m.I_max)
        ok, have, need = battery_can_feed(fp["capacity_Ah"], fp["c_rating"],
                                          fp["n_motors"], weakest.I_max)
        self.assertFalse(ok)
        self.assertAlmostEqual(have, 18.0)
        self.assertGreaterEqual(need, 128.0)


class Engine(unittest.TestCase):

    def test_every_paper_payload_gets_an_answer(self):
        # Either a design that passes every check, or a clear reason why not.
        for c in PAPER_CASES:
            with self.subTest(payload=c.payload_g):
                d = size(Requirements(c.payload_g, 15, c.n_motors))
                self.assertTrue(d.feasible or d.problems)

    def test_designs_obey_the_rules(self):
        for payload in (1000, 2000, 5000, 10000):
            with self.subTest(payload=payload):
                req = Requirements(payload, 15)
                d = size(req)
                self.assertTrue(d.feasible, d.problems)
                m = d.motor
                # weight settled: the masses add up to the AUW used for sizing
                self.assertAlmostEqual(sum(d.masses_g.values()), d.auw_g, delta=1.0)
                # 2:1 thrust
                self.assertGreaterEqual(d.n_motors * m.T_max_g, 2 * d.auw_g - 1e-6)
                # ESC: motor peak at most 80% of rating
                self.assertLessEqual(m.I_max, 0.8 * d.esc_A + 1e-9)
                # battery can feed every motor at full throttle
                self.assertGreaterEqual(d.c_rating * d.capacity_Ah, d.I_max_total)
                # hover for the time asked, on 80% of the pack
                self.assertGreaterEqual(d.endurance_min, req.endurance_min - 0.05)

    def test_more_endurance_means_more_battery(self):
        short = size(Requirements(2000, 10, 4))
        long = size(Requirements(2000, 20, 4))
        self.assertGreater(long.masses_g["battery"], short.masses_g["battery"])
        self.assertGreater(long.auw_g, short.auw_g)

    def test_impossible_request_says_why(self):
        # The paper claims 45 minutes with 6 kg; this catalogue cannot do it.
        d = size(Requirements(6000, 45))
        self.assertFalse(d.feasible)
        self.assertTrue(any("thrust" in p or "runs away" in p for p in d.problems))

    def test_cost_matches_table_4_at_its_own_payloads(self):
        self.assertAlmostEqual(cost_inr_2022(2000)["total"], 54300)
        self.assertAlmostEqual(cost_inr_2022(30000)["total"], 270650)
        mid = cost_inr_2022(3500)["total"]
        self.assertTrue(54300 < mid < 96300)


if __name__ == "__main__":
    unittest.main()
