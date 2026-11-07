"""Synthetic radar temporal evaluation has explicit false positives."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from benchmarks.radar_temporal_benchmark import evaluate


class RadarTemporalBenchmarkTests(unittest.TestCase):
    def test_labeled_confusions_and_adversarial_blind_spots(self):
        result = evaluate()
        self.assertEqual(result["scenario_count"], 180)
        self.assertFalse(result["measured_physical_accuracy"])
        instant = result["strategies"]["single_frame_any"]
        temporal = result["strategies"]["three_independent_updates"]
        self.assertEqual(instant["confusion"]["fp"], 60)
        self.assertEqual(temporal["confusion"]["fp"], 40)
        self.assertEqual(temporal["confusion"]["tp"], 40)
        self.assertAlmostEqual(instant["precision"], .4)
        self.assertAlmostEqual(temporal["precision"], .5)
        self.assertAlmostEqual(temporal["recall"], 1.)
        self.assertEqual(
            result["per_class"]["negative_aligned_reflection"]
            ["three_independent_updates"]["fp"], 20,
        )
        self.assertEqual(
            result["per_class"]["negative_aligned_impostor"]
            ["three_independent_updates"]["fp"], 20,
        )

    def test_negative_transient_is_not_temporally_correlated(self):
        scenario = evaluate()["per_class"]["negative_one_frame_coincidence"]
        self.assertEqual(scenario["single_frame_any"]["fp"], 20)
        self.assertEqual(scenario["three_independent_updates"]["tn"], 20)


if __name__ == "__main__":
    unittest.main()
