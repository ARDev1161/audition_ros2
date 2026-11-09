"""Paired M4 comparison must not mistake abstention or unknown truth for success."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from paired_comparison import compare, correct


class PairedComparisonTests(unittest.TestCase):
    def test_true_negative_abstention_and_wrong_pair(self):
        truth = ("a", "r")
        self.assertTrue(correct(truth, ("pair", truth)))
        self.assertFalse(correct(truth, ("none", None)))
        self.assertFalse(correct(truth, ("pair", ("a", "impostor"))))
        self.assertTrue(correct(None, ("none", None)))
        self.assertFalse(correct(None, ("abstain", None)))
        self.assertFalse(correct(None, ("pair", truth)))

    def test_paired_win_loss_and_per_stratum(self):
        truth = ("a", "r")
        impostor = ("a", "wrong")
        rows = [
            # candidate wins over instant; radar abstention also loses
            ("t1", "clear", "held_out", True, truth, {
                "audio_only": ("pair", truth), "radar_only": ("abstain", None),
                "instant_fusion": ("pair", impostor),
                "temporal_fusion": ("pair", truth),
            }),
            # candidate loses on a negative: false association
            ("t2", "impostor", "held_out", True, None, {
                "audio_only": ("none", None), "radar_only": ("none", None),
                "instant_fusion": ("none", None),
                "temporal_fusion": ("pair", impostor),
            }),
            # candidate wins on a negative
            ("t3", "impostor", "held_out", True, None, {
                "audio_only": ("pair", impostor), "radar_only": ("pair", impostor),
                "instant_fusion": ("pair", impostor),
                "temporal_fusion": ("none", None),
            }),
            # Unknown truth must not affect either numerator or denominator
            ("t4", "unknown", "held_out", False, None, {
                name: ("pair", impostor) for name in (
                    "audio_only", "radar_only", "instant_fusion", "temporal_fusion")
            }),
        ]
        result = compare(rows)
        d = result["baseline_comparisons"]["instant_fusion"]
        self.assertEqual(d["observable_trials"], 3)
        self.assertEqual(d["candidate_only_correct"], 2)
        self.assertEqual(d["baseline_only_correct"], 1)
        self.assertEqual(d["both_correct"], 0)
        self.assertEqual(d["unobservable_excluded"], 1)
        self.assertAlmostEqual(d["paired_accuracy_delta"], 1/3)
        self.assertEqual(d["candidate_false_pairs"], 1)
        self.assertEqual(d["baseline_false_pairs"], 2)
        self.assertAlmostEqual(d["paired_false_pair_delta_per_trial"], -1/3)
        self.assertTrue(d["descriptive_improvement"])
        self.assertEqual(result["per_stratum"]["impostor"]["instant_fusion"]
                         ["baseline_only_correct"], 1)
        self.assertFalse(result["positive_gain_validated"])

    def test_all_unknown_yields_null_not_fabricated_gain(self):
        p = {name: ("abstain", None) for name in (
            "audio_only", "radar_only", "instant_fusion", "temporal_fusion")}
        result = compare([("unknown", "occluded", "held_out", False, None, p)])
        for d in result["baseline_comparisons"].values():
            self.assertEqual(d["unobservable_excluded"], 1)
            self.assertIsNone(d["paired_accuracy_delta"])
            self.assertIsNone(d["paired_false_pair_delta_per_trial"])
            self.assertIsNone(d["descriptive_improvement"])

    def test_identical_methods_have_zero_gain(self):
        p = {name: ("none", None) for name in (
            "audio_only", "radar_only", "instant_fusion", "temporal_fusion")}
        result = compare([("negative", "direct", "held_out", True, None, p)])
        for d in result["baseline_comparisons"].values():
            self.assertEqual(d["both_correct"], 1)
            self.assertEqual(d["paired_accuracy_delta"], 0)
            self.assertFalse(d["descriptive_improvement"])


if __name__ == "__main__":
    unittest.main()
