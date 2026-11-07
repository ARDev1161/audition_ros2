"""M4 end-to-end diagnostic admission without ROS or fabricated covariance."""
import math
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from diagnostic_pipeline import evaluate_disjoint_windows
from uncertainty import PlanarRay

SESSION = "a" * 32


def window(start, target=(3.0, 2.0), pose_var=0.01, angle_var=0.001):
    output = []
    for i, x in enumerate((0.0, 1.0, 2.0)):
        dx, dy = target[0] - x, target[1]
        n = math.hypot(dx, dy)
        output.append(PlanarRay(start + i * 200_000_000,
            (x, 0.0), (dx / n, dy / n), angle_var,
            (pose_var, 0.0, 0.0, pose_var)))
    return output


class DiagnosticPipelineTests(unittest.TestCase):
    def setUp(self):
        self.first = window(1_000_000_000)
        self.second = window(2_000_000_000)

    def run_gate(self, first=None, second=None):
        return evaluate_disjoint_windows(
            SESSION, "source", 1,
            self.first if first is None else first,
            self.second if second is None else second)

    def test_admits_consistent_independent_windows(self):
        value = self.run_gate()
        self.assertIsNotNone(value)
        self.assertEqual(value.first.candidate.observations, 3)
        self.assertEqual(value.second.covariance.supporting_observations, 3)
        self.assertAlmostEqual(value.stability.x, 3.0)
        self.assertAlmostEqual(value.stability.y, 2.0)

    def test_missing_pose_covariance_fails_closed(self):
        second = self.second[:]
        second[1] = replace(second[1], origin_covariance=None)
        self.assertIsNone(self.run_gate(second=second))

    def test_overlap_and_replayed_window_fails(self):
        self.assertIsNone(self.run_gate(second=self.first))
        self.assertIsNone(self.run_gate(second=window(1_500_000_000)))

    def test_moving_source_in_second_window_fails(self):
        self.assertIsNone(self.run_gate(
            second=window(2_000_000_000, target=(5.0, 2.0))))

    def test_bad_bearings_and_insufficient_observations(self):
        self.assertIsNone(self.run_gate(first=self.first[:2]))
        changed = self.second[:]
        changed[2] = replace(changed[2], direction=(1.0, 0.0))
        self.assertIsNone(self.run_gate(second=changed))

    def test_no_geometry_from_stationary_array(self):
        changed = window(2_000_000_000)
        changed = [replace(r, origin=(0.0, 0.0)) for r in changed]
        self.assertIsNone(self.run_gate(second=changed))

    def test_nan_and_session_validation(self):
        self.assertIsNone(evaluate_disjoint_windows(
            "wrong", "source", 1, self.first, self.second))
        changed = self.second[:]
        changed[0] = replace(changed[0], angular_variance_rad2=float("nan"))
        self.assertIsNone(self.run_gate(second=changed))


if __name__ == "__main__":
    unittest.main()
