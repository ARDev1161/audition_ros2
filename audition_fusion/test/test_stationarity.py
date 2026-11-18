"""M4 independent-window consistency admission regressions."""
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from stationarity import Fix, compare_disjoint_fixes


def fix(start, end, x=3.0, y=2.0, sd=0.2):
    return Fix("a"*32, "source", 1, start, end, x, y, sd)


class StationarityTests(unittest.TestCase):
    def setUp(self):
        self.a = fix(1_000_000_000, 1_400_000_000)
        self.b = fix(1_700_000_000, 2_100_000_000, 3.1, 2.1)

    def test_consistent_disjoint_windows(self):
        result = compare_disjoint_fixes(self.a, self.b)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.x, 3.05)
        self.assertAlmostEqual(result.y, 2.05)
        self.assertEqual(result.temporal_separation_ns, 300_000_000)

    def test_overlap_and_reused_boundary_rejected(self):
        for start in (1_200_000_000, 1_400_000_000, 1_500_000_000):
            self.assertIsNone(compare_disjoint_fixes(
                self.a, replace(self.b, first_stamp_ns=start)))
        self.assertIsNone(compare_disjoint_fixes(self.b, self.a))

    def test_motion_and_disagreement_beyond_noise_rejected(self):
        self.assertIsNone(compare_disjoint_fixes(
            self.a, replace(self.b, x=4.0)))
        self.assertIsNone(compare_disjoint_fixes(
            replace(self.a, major_std_m=0.001),
            replace(self.b, major_std_m=0.001)))

    def test_identity_generation_and_session_fences(self):
        for change in (
            {"session": "b"*32},
            {"source_id": "other"},
            {"generation": 2},
        ):
            self.assertIsNone(compare_disjoint_fixes(
                self.a, replace(self.b, **change)))

    def test_invalid_time_and_uncertainty_fail_closed(self):
        for updated in (
            replace(self.b, major_std_m=0),
            replace(self.b, major_std_m=float("nan")),
            replace(self.b, x=float("inf")),
            replace(self.b, first_stamp_ns=0),
            replace(self.b, last_stamp_ns=self.b.first_stamp_ns),
        ):
            self.assertIsNone(compare_disjoint_fixes(self.a, updated))
        self.assertIsNone(compare_disjoint_fixes(
            self.a, self.b, maximum_disagreement_m=float("nan")))


if __name__ == "__main__":
    unittest.main()
