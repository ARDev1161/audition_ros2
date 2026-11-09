"""Common-mode errors can be large while the conditional gate still admits."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common_mode_audit import assert_expected, evaluate_case, report


class CommonModeAuditTests(unittest.TestCase):
    def test_adversarial_geometry_contract(self):
        result = report()
        assert_expected(result)
        self.assertTrue(result["cases"]["coherent_reflection_image"]["admitted"])
        self.assertFalse(result["metric_position_published"])

    def test_bias_changes_error_not_conditional_uncertainty(self):
        clean = evaluate_case()
        half = evaluate_case(bias=(0.5, 0.0))
        full = evaluate_case(bias=(1.0, 0.0))
        self.assertTrue(clean["admitted"])
        self.assertTrue(half["admitted"])
        self.assertTrue(full["admitted"])
        self.assertAlmostEqual(clean["position_error_m"], 0.0, places=6)
        self.assertAlmostEqual(half["position_error_m"], .5, places=6)
        self.assertAlmostEqual(full["position_error_m"], 1.0, places=6)
        self.assertAlmostEqual(
            clean["conditional_major_std_m"],
            full["conditional_major_std_m"], places=6)

    def test_correlated_y_bias_is_also_undetected(self):
        result = evaluate_case(bias=(0.0, -0.75))
        self.assertTrue(result["admitted"])
        self.assertAlmostEqual(result["position_error_m"], 0.75, places=6)

    def test_reflection_is_not_independent_source_truth(self):
        reflection = evaluate_case(apparent_source=(3.0, -2.0))
        self.assertTrue(reflection["admitted"])
        self.assertAlmostEqual(reflection["position_error_m"], 4.0, places=6)
        self.assertGreater(reflection["error_to_conditional_std_ratio"], 1)


if __name__ == "__main__":
    unittest.main()
