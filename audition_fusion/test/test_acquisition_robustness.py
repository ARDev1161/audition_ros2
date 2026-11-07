"""Cross-checked acquisition group robustness diagnostics: no synthetic field claim."""
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "benchmarks" / "evaluate_field_manifest.py"


def synthetic_case(*, lone_gain=False, one_regressing_group=False,
                   same_scene=False, only_unknown=False):
    good = {"audio_id": "speaker", "radar_id": "r1"}
    bad = {"audio_id": "speaker", "radar_id": "ghost"}
    true_prediction = {"decision": "pair", "pair": good}
    wrong_prediction = {"decision": "pair", "pair": bad}
    trials, provenance = [], []
    for i in range(5):
        baseline = (true_prediction if (lone_gain and i != 0
                                        or one_regressing_group and i == 0)
                    else wrong_prediction)
        candidate = (wrong_prediction if one_regressing_group and i == 0
                     else true_prediction)
        digest = format(i + 1, "064x")
        tid = "case-" + str(i)
        trials.append({
            "trial_id": tid, "stratum": "direct", "split": "held_out",
            "recording_sha256": digest,
            "truth": {"observable": not only_unknown,
                      "pair": None if only_unknown else good},
            "predictions": {
                "audio_only": baseline.copy(), "radar_only": baseline.copy(),
                "instant_fusion": baseline.copy(),
                "temporal_fusion": candidate.copy(),
            },
        })
        provenance.append({
            "trial_id": tid, "split": "held_out",
            "recording_sha256": digest,
            "session_id": "session-" + str(i),
            "scene_id": "shared-room" if same_scene else "scene-" + str(i),
            "subject_id": "emitter-" + str(i),
        })
    return ({"schema_version": 1, "dataset_id": "acquisition-robustness-fixture",
             "trials": trials}, {"trials": provenance})


class AcquisitionRobustnessTests(unittest.TestCase):
    def run_cli(self, **kwargs):
        dataset, provenance = synthetic_case(**kwargs)
        with tempfile.TemporaryDirectory() as dirname:
            folder = pathlib.Path(dirname)
            manifest = folder / "trials.json"
            audit = folder / "acquisition.json"
            manifest.write_text(json.dumps(dataset), encoding="utf-8")
            audit.write_text(json.dumps(provenance), encoding="utf-8")
            proc = subprocess.run(
                [sys.executable, str(SCRIPT), str(manifest),
                 "--acquisition-provenance", str(audit)],
                text=True, capture_output=True, check=False)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            return json.loads(proc.stdout)

    def test_five_distinct_groups_support_positive_descriptive_loo(self):
        report = self.run_cli()
        audit = report["acquisition_provenance_audit"]
        sensitivity = audit["paired_group_sensitivity"]
        self.assertFalse(sensitivity["physical_validation_claim"])
        for dimension in ("session_id", "scene_id", "subject_id"):
            stats = sensitivity["per_stratum"]["direct"]["dimensions"][
                dimension]["baselines"]["instant_fusion"]
            self.assertEqual(stats["acquisition_groups"], 5)
            self.assertEqual(stats["observable_trials"], 5)
            self.assertEqual(stats["paired_accuracy_delta"], 1.0)
            self.assertEqual(stats["leave_one_group_out_accuracy_range"], [1.0, 1.0])
            self.assertEqual(stats["leave_one_group_out_false_pair_range"], [-1.0, -1.0])
        # Positive conditional LOO is not external proof of a field domain.
        # The CLI must reject promotion when dataset origin is unverified.
        self.assertIn("data_domain_unverified", report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertFalse(report["safety_diagnostic"]["physical_localization_accepted"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_pooled_gain_disappears_when_only_useful_group_is_removed(self):
        report = self.run_cli(lone_gain=True)
        stats = report["acquisition_provenance_audit"]["paired_group_sensitivity"][
            "per_stratum"]["direct"]["dimensions"]["session_id"][
            "baselines"]["instant_fusion"]
        self.assertGreater(stats["paired_accuracy_delta"], 0)
        self.assertEqual(stats["leave_one_group_out_accuracy_range"][0], 0)
        self.assertIn(
            "fragile_acquisition_gain:direct:session_id:instant_fusion",
            report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])

    def test_adverse_group_is_not_hidden_by_four_better_groups(self):
        report = self.run_cli(one_regressing_group=True)
        stats = report["acquisition_provenance_audit"]["paired_group_sensitivity"][
            "per_stratum"]["direct"]["dimensions"]["scene_id"][
            "baselines"]["instant_fusion"]
        self.assertGreater(stats["paired_accuracy_delta"], 0)
        self.assertLess(stats["paired_false_pair_delta_per_trial"], 0)
        self.assertGreater(stats["max_single_group_false_pair_delta_per_trial"], 0)
        self.assertLess(stats["min_single_group_accuracy_delta"], 0)
        blockers = report["safety_diagnostic"]["blockers"]
        self.assertIn(
            "acquisition_group_false_pair_regression:direct:scene_id:instant_fusion",
            blockers)
        self.assertIn(
            "acquisition_group_accuracy_regression:direct:scene_id:instant_fusion",
            blockers)
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])

    def test_five_files_same_room_have_no_scene_loo(self):
        report = self.run_cli(same_scene=True)
        stats = report["acquisition_provenance_audit"]["paired_group_sensitivity"][
            "per_stratum"]["direct"]["dimensions"]["scene_id"][
            "baselines"]["instant_fusion"]
        self.assertEqual(stats["acquisition_groups"], 1)
        self.assertIsNone(stats["leave_one_group_out_accuracy_range"])
        self.assertIn(
            "insufficient_acquisition_groups:direct:scene_id",
            report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])

    def test_unknown_truth_contributes_no_robustness_evidence(self):
        report = self.run_cli(only_unknown=True)
        sensitivity = report["acquisition_provenance_audit"]["paired_group_sensitivity"]
        data = sensitivity["per_stratum"]["direct"]
        self.assertEqual(data["unobservable_excluded"], 5)
        stats = data["dimensions"]["session_id"]["baselines"]["instant_fusion"]
        self.assertEqual(stats["acquisition_groups"], 0)
        self.assertIsNone(stats["paired_accuracy_delta"])
        self.assertIsNone(stats["leave_one_group_out_accuracy_range"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])


if __name__ == "__main__":
    unittest.main()
