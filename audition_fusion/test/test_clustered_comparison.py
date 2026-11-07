"""Independent recording clusters, deterministic sensitivity, no false confidence."""
import copy
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from field_evaluation import evaluate


def trial(i, *, digest=None, split="held_out", observed=True,
          correct_temporal=True):
    real = {"audio_id": "a", "radar_id": "r"}
    wrong = {"audio_id": "a", "radar_id": "wrong"}
    return {
        "trial_id": "trial-" + str(i),
        "stratum": "direct",
        "split": split,
        "recording_sha256": digest or (format(i, "064x")),
        "truth": {"observable": observed, "pair": real if observed else None},
        "predictions": {
            "audio_only": {"decision": "pair", "pair": wrong},
            "radar_only": {"decision": "abstain", "pair": None},
            "instant_fusion": {"decision": "pair", "pair": wrong},
            "temporal_fusion": {"decision": "pair",
                                "pair": real if correct_temporal else wrong},
        },
    }


def dataset(items):
    return {"schema_version": 1, "dataset_id": "cluster-fixture", "trials": items}


class ClusteredComparisonTests(unittest.TestCase):
    def test_single_recording_many_events_cannot_yield_interval(self):
        rows = [trial(i, digest="a" * 64) for i in range(1, 21)]
        result = evaluate(dataset(rows))["clustered_comparison"]
        d = result["baselines"]["instant_fusion"]
        self.assertEqual(d["recording_clusters"], 1)
        self.assertEqual(d["observable_trials"], 20)
        self.assertIsNone(d["bootstrap_95pct_accuracy_interval"])
        self.assertIsNone(d["leave_one_recording_out_accuracy_range"])
        self.assertFalse(result["physical_validation_claim"])

    def test_five_recordings_bootstrap_deterministic_and_loo(self):
        rows = [trial(i, correct_temporal=i != 5) for i in range(1, 6)]
        a = evaluate(dataset(rows))["clustered_comparison"]["baselines"]["instant_fusion"]
        b = evaluate(dataset(copy.deepcopy(rows)))["clustered_comparison"]["baselines"]["instant_fusion"]
        self.assertEqual(a, b)
        self.assertEqual(a["recording_clusters"], 5)
        self.assertEqual(a["paired_accuracy_delta"], 0.8)
        self.assertEqual(a["paired_false_pair_delta_per_trial"], -0.8)
        self.assertEqual(a["leave_one_recording_out_accuracy_range"], [0.75, 1.0])
        self.assertEqual(a["leave_one_recording_out_false_pair_range"], [-1.0, -0.75])
        lo, hi = a["bootstrap_95pct_accuracy_interval"]
        self.assertLessEqual(lo, 0.8)
        self.assertGreaterEqual(hi, 0.8)
        self.assertTrue(0 <= lo <= hi <= 1)
        self.assertEqual(a["bootstrap_draws"], 512)

    def test_held_out_and_unknown_excluded(self):
        rows = [trial(1), trial(2, split="development"),
                trial(3, observed=False)]
        result = evaluate(dataset(rows))["clustered_comparison"]
        d = result["baselines"]["audio_only"]
        self.assertEqual(d["recording_clusters"], 1)
        self.assertEqual(d["observable_trials"], 1)
        self.assertEqual(result["unobservable_excluded"], 1)
        self.assertIsNone(d["bootstrap_95pct_accuracy_interval"])
        self.assertEqual(d["paired_accuracy_delta"], 1.0)

    def test_all_unknown_has_no_inferred_gain(self):
        result = evaluate(dataset([trial(1, observed=False)]))["clustered_comparison"]
        for d in result["baselines"].values():
            self.assertEqual(d["recording_clusters"], 0)
            self.assertIsNone(d["paired_accuracy_delta"])
            self.assertIsNone(d["bootstrap_95pct_accuracy_interval"])

    def test_repeated_same_recording_not_independent(self):
        rows = [trial(1, digest="a" * 64),
                trial(2, digest="a" * 64),
                trial(3, digest="b" * 64)]
        report = evaluate(dataset(rows))["clustered_comparison"]
        d = report["baselines"]["radar_only"]
        self.assertEqual(d["recording_clusters"], 2)
        self.assertEqual(d["min_observable_trials_per_recording"], 1)
        self.assertEqual(d["max_observable_trials_per_recording"], 2)


    def test_strata_preserve_cluster_counts_and_unknowns(self):
        rows = [trial(i) for i in range(1, 7)]
        for row in rows[3:]:
            row["stratum"] = "reflection"
            row["predictions"]["temporal_fusion"] = {
                "decision": "pair",
                "pair": {"audio_id": "a", "radar_id": "wrong"},
            }
        unknown = trial(7, observed=False)
        unknown["stratum"] = "unknown"
        rows.append(unknown)
        result = evaluate(dataset(rows))["clustered_comparison"]
        self.assertEqual(result["baselines"]["instant_fusion"]["recording_clusters"], 6)
        direct = result["per_stratum"]["direct"]["baselines"]["instant_fusion"]
        reflection = result["per_stratum"]["reflection"]["baselines"]["instant_fusion"]
        self.assertEqual(direct["recording_clusters"], 3)
        self.assertEqual(reflection["recording_clusters"], 3)
        self.assertEqual(direct["paired_accuracy_delta"], 1.0)
        self.assertEqual(reflection["paired_accuracy_delta"], 0.0)
        self.assertIsNone(direct["bootstrap_95pct_accuracy_interval"])
        self.assertIsNone(reflection["bootstrap_95pct_accuracy_interval"])
        self.assertEqual(result["per_stratum"]["unknown"]["unobservable_excluded"], 1)
        self.assertEqual(result["per_stratum"]["unknown"]["baselines"]["audio_only"]
                         ["recording_clusters"], 0)

    def test_one_recording_in_multiple_strata_is_one_overall_cluster(self):
        rows = [trial(1, digest="c" * 64), trial(2, digest="c" * 64)]
        rows[1]["stratum"] = "reflection"
        result = evaluate(dataset(rows))["clustered_comparison"]
        self.assertEqual(result["baselines"]["audio_only"]["recording_clusters"], 1)
        self.assertEqual(result["per_stratum"]["direct"]["baselines"]["audio_only"]
                         ["recording_clusters"], 1)
        self.assertEqual(result["per_stratum"]["reflection"]["baselines"]["audio_only"]
                         ["recording_clusters"], 1)


if __name__ == "__main__":
    unittest.main()
