"""Diagnostic safety blockers cannot be hidden by pooled association gain."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from field_evaluation import evaluate
from safety_diagnostic import provenance_blockers


def trial(i, stratum, *, wrong=False, digest=None, observable=True):
    good = {"audio_id": "source", "radar_id": "radar"}
    bad = {"audio_id": "source", "radar_id": "impostor"}
    correct = {"decision": "pair", "pair": good}
    incorrect = {"decision": "pair", "pair": bad}
    return {
        "trial_id": "t" + str(i),
        "stratum": stratum,
        "split": "held_out",
        "recording_sha256": digest or format(i, "064x"),
        "truth": {"observable": observable, "pair": good if observable else None},
        "predictions": {
            "audio_only": incorrect.copy(),
            "radar_only": incorrect.copy(),
            "instant_fusion": incorrect.copy(),
            "temporal_fusion": (incorrect if wrong else correct).copy(),
        },
    }


def evaluate_rows(rows):
    return evaluate({"schema_version": 1, "dataset_id": "safety-regression",
                     "trials": rows})["safety_diagnostic"]


class SafetyDiagnosticTests(unittest.TestCase):
    def test_provenance_rejects_session_scene_and_subject_leakage(self):
        a = {"trial_id": "a", "split": "development", "recording_sha256": "a"*64,
             "session_id": "session1", "scene_id": "room1", "subject_id": "person1"}
        b = dict(a, trial_id="b", split="held_out", recording_sha256="b"*64)
        problems = provenance_blockers({"trials": [a, b]})
        for field in ("session_id", "scene_id", "subject_id"):
            self.assertIn("provenance_split_leak:" + field, problems)

    def test_provenance_rejects_missing_and_duplicate_trials(self):
        a = {"trial_id": "a", "split": "held_out", "recording_sha256": "a"*64,
             "session_id": "", "scene_id": "room1", "subject_id": "person1"}
        problems = provenance_blockers({"trials": [a, a]})
        self.assertIn("provenance_missing:session_id", problems)
        self.assertIn("provenance_trial_duplicate_or_invalid", problems)
        self.assertIn("provenance_manifest_missing_or_invalid", provenance_blockers(None))

    def test_provenance_allows_distinct_sessions_and_subjects(self):
        rows = [dict(trial_id=str(i), split=split, recording_sha256=hex(i+1)[2:].zfill(64),
                     session_id="session"+str(i), scene_id="room"+str(i),
                     subject_id="person"+str(i)) for i, split in enumerate(("development", "held_out"))]
        self.assertEqual(provenance_blockers({"trials": rows}), [])

    def test_pooled_gain_does_not_hide_adverse_false_pair_regression(self):
        rows = [trial(i, "direct") for i in range(1, 7)]
        rows.append(trial(7, "reflection", wrong=True))
        # In reflection the instant baseline has a verified correct pair;
        # temporal fusion instead emits a wrong radar association.
        rows[-1]["predictions"]["instant_fusion"] = {
            "decision": "pair", "pair": {"audio_id": "source", "radar_id": "radar"}}
        safety = evaluate_rows(rows)
        self.assertIn("false_pair_regression:reflection:instant_fusion",
                      safety["blockers"])
        self.assertFalse(safety["no_observed_safety_regression"])
        self.assertFalse(safety["diagnostic_promotion_eligible"])
        self.assertFalse(safety["physical_localization_accepted"])
        self.assertFalse(safety["nav2_admission"])

    def test_five_clean_recordings_do_not_imply_physical_acceptance(self):
        rows = [trial(i, "direct") for i in range(1, 6)]
        safety = evaluate_rows(rows)
        self.assertEqual(safety["blockers"], [])
        self.assertTrue(safety["diagnostic_promotion_eligible"])
        self.assertTrue(safety["no_observed_safety_regression"])
        self.assertFalse(safety["physical_localization_accepted"])
        self.assertFalse(safety["nav2_admission"])

    def test_single_recording_and_unknown_truth_fail_closed(self):
        rows = [trial(i, "direct", digest="a" * 64) for i in range(1, 10)]
        rows.append(trial(10, "occluded", observable=False))
        safety = evaluate_rows(rows)
        self.assertTrue(any(s.startswith("insufficient_recording_clusters:direct:")
                            for s in safety["blockers"]))
        self.assertTrue(any(s.startswith("unknown_truth:occluded:")
                            for s in safety["blockers"]))
        self.assertFalse(safety["diagnostic_promotion_eligible"])
        self.assertFalse(safety["no_observed_safety_regression"])

    def test_insufficient_clusters_cannot_claim_clean_safety(self):
        rows = [trial(i, "direct", digest="a" * 64) for i in range(1, 7)]
        safety = evaluate_rows(rows)
        self.assertTrue(any(x.startswith("insufficient_recording_clusters:")
                            for x in safety["blockers"]))
        self.assertFalse(safety["no_observed_safety_regression"])
        self.assertFalse(safety["diagnostic_promotion_eligible"])
        self.assertFalse(safety["nav2_admission"])

    def test_no_pooled_gain_cannot_claim_clean_safety(self):
        rows = [trial(i, "direct") for i in range(1, 7)]
        for row in rows:
            row["predictions"]["audio_only"] = row["predictions"]["temporal_fusion"].copy()
        safety = evaluate_rows(rows)
        self.assertIn("no_pooled_accuracy_gain:audio_only", safety["blockers"])
        self.assertFalse(safety["no_observed_safety_regression"])

    def test_partially_unobservable_stratum_blocks_clean_safety_claim(self):
        rows = [trial(i, "mixed") for i in range(1, 7)]
        rows.append(trial(7, "mixed", observable=False))
        safety = evaluate_rows(rows)
        self.assertIn("unobservable_ground_truth:mixed", safety["blockers"])
        self.assertFalse(safety["no_observed_safety_regression"])
        self.assertFalse(safety["diagnostic_promotion_eligible"])
        self.assertFalse(safety["physical_localization_accepted"])
        self.assertFalse(safety["nav2_admission"])

    def test_unknown_only_stratum_never_claims_no_regression(self):
        rows = [trial(i, "direct") for i in range(1, 6)]
        rows += [trial(i, "unobservable", observable=False) for i in range(6, 11)]
        safety = evaluate_rows(rows)
        self.assertIn("unknown_truth:unobservable:audio_only", safety["blockers"])
        self.assertFalse(safety["no_observed_safety_regression"])
        self.assertFalse(safety["diagnostic_promotion_eligible"])
        self.assertFalse(safety["nav2_admission"])

    def test_development_split_is_never_diagnostic_promotion(self):
        rows = [trial(i, "direct") for i in range(1, 6)]
        rows[0]["split"] = "development"
        safety = evaluate({"schema_version": 1, "dataset_id": "safety-regression",
                           "trials": rows}, held_out_only=False)["safety_diagnostic"]
        self.assertIn("development_data_included", safety["blockers"])


if __name__ == "__main__":
    unittest.main()
