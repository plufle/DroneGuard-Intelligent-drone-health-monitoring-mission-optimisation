"""The planning website's API (P3). Needs the web extras:

    .venv/Scripts/python -m unittest -v
"""

import unittest

try:
    from fastapi.testclient import TestClient
    from web.app import app
except ImportError:          # plain Python without FastAPI: skip, don't fail
    app = None


@unittest.skipIf(app is None, "FastAPI not installed (pip install -r requirements.txt)")
class PlannerAPI(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.client = TestClient(app)

    def plan(self, **body):
        r = self.client.post("/api/plan", json=body)
        self.assertEqual(r.status_code, 200, r.text)
        return r.json()

    def test_page_and_presets(self):
        self.assertIn("DroneGuard", self.client.get("/").text)
        apps = self.client.get("/api/applications").json()
        delivery = next(a for a in apps if a["key"] == "delivery")
        self.assertTrue(delivery["available"])
        self.assertEqual(delivery["payload_kg"]["default"], 2)

    def test_default_delivery_is_go(self):
        p = self.plan()
        self.assertTrue(p["design"]["feasible"])
        self.assertEqual(p["check"]["verdict"], "GO")
        self.assertAlmostEqual(p["cost_inr"]["total"], 54300)
        s = p["series"]["stressed"]
        self.assertEqual(len(s["t"]), len(s["overall"]))
        self.assertLessEqual(len(s["t"]), 400)

    def test_too_far_is_no_go(self):
        p = self.plan(distance_km=8)
        self.assertEqual(p["check"]["verdict"], "NO-GO")

    def test_paper_delivery_claim_cannot_be_built(self):
        p = self.plan(payload_kg=6, endurance_min=45)
        self.assertFalse(p["design"]["converged"])
        self.assertIsNone(p["check"])
        self.assertTrue(p["design"]["problems"])

    def test_bad_inputs_are_rejected(self):
        self.assertEqual(self.client.post("/api/plan", json={"payload_kg": -1}).status_code, 422)
        self.assertEqual(self.client.post("/api/plan", json={"motors": 5}).status_code, 422)
        self.assertEqual(self.client.post("/api/plan", json={"application": "agriculture"}).status_code, 400)


if __name__ == "__main__":
    unittest.main()
