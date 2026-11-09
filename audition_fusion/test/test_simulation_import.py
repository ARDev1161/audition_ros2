"""Simulator export v1: no fabricated labels, no unverified recording hashes."""
import copy
import hashlib
import pathlib
import tempfile
import unittest
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from simulation_import import convert
from field_evaluation import evaluate

PAIR = {"audio_id": "speaker-1", "radar_id": "object-1"}


def pred(decision, pair=None):
    return {"decision": decision, "pair": pair}


def trial(tid="trial-1"):
    return {
        "trial_id": tid,
        "stratum": "direct",
        "truth_source": "simulator_authoritative",
        "truth": {"observable": True, "pair": PAIR},
        "predictions": {
            "audio_only": pred("abstain"),
            "radar_only": pred("abstain"),
            "instant_fusion": pred("pair", PAIR),
            "temporal_fusion": pred("pair", PAIR),
        },
    }


def export(sha):
    return {
        "format": "audition.simulation-export.v1",
        "domain": "simulation", "dataset_id": "scene-table",
        "scenarios": [{
            "scenario_id": "seven-speakers",
            "split": "held_out",
            "recording": "scene.bin",
            "recording_sha256": sha,
            "trials": [trial()],
        }],
    }


class SimulationExportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.file = self.root / "scene.bin"
        self.file.write_bytes(b"synthetic fixture, no actual audio")
        self.sha = hashlib.sha256(self.file.read_bytes()).hexdigest()

    def test_import_and_scoring(self):
        manifest = convert(export(self.sha), artifacts_root=self.root)
        self.assertEqual(manifest["schema_version"], 1)
        self.assertEqual(manifest["trials"][0]["recording_sha256"], self.sha)
        result = evaluate(manifest)
        self.assertEqual(result["strategies"]["instant_fusion"]["tp"], 1)
        self.assertFalse(result["physical_validation_claim"])

    def test_wrong_hash_and_path_traversal_are_rejected(self):
        bad = export("0" * 64)
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            convert(bad, artifacts_root=self.root)
        bad = export(self.sha)
        bad["scenarios"][0]["recording"] = "../scene.bin"
        with self.assertRaisesRegex(ValueError, "relative recording"):
            convert(bad, artifacts_root=self.root)

    def test_no_truth_fabrication_and_no_incomplete_method_output(self):
        bad = export(self.sha)
        bad["scenarios"][0]["trials"][0]["truth_source"] = "inferred_from_doa"
        with self.assertRaisesRegex(ValueError, "simulator authority"):
            convert(bad, artifacts_root=self.root)
        bad = export(self.sha)
        del bad["scenarios"][0]["trials"][0]["predictions"]["radar_only"]
        with self.assertRaisesRegex(ValueError, "comparison strategies"):
            convert(bad, artifacts_root=self.root)

    def test_no_cross_split_recording_leakage(self):
        bad = export(self.sha)
        second = copy.deepcopy(bad["scenarios"][0])
        second["scenario_id"] = "second-scene"
        second["split"] = "development"
        second["trials"] = [trial("trial-2")]
        bad["scenarios"].append(second)
        with self.assertRaisesRegex(ValueError, "development and held_out"):
            convert(bad, artifacts_root=self.root)

    def test_duplicate_scenarios_and_duplicate_trials(self):
        bad = export(self.sha)
        bad["scenarios"].append(copy.deepcopy(bad["scenarios"][0]))
        with self.assertRaisesRegex(ValueError, "duplicate scenario_id"):
            convert(bad, artifacts_root=self.root)
        bad = export(self.sha)
        bad["scenarios"][0]["trials"].append(trial())
        with self.assertRaisesRegex(ValueError, "duplicate trial_id"):
            convert(bad, artifacts_root=self.root)

    def test_unobservable_truth_remains_unscored(self):
        data = export(self.sha)
        source = data["scenarios"][0]["trials"][0]
        source["truth"] = {"observable": False, "pair": None}
        result = evaluate(convert(data, artifacts_root=self.root))
        stats = result["strategies"]["temporal_fusion"]
        self.assertEqual(stats["unobservable"], 1)
        self.assertIsNone(stats["precision"])


if __name__ == "__main__":
    unittest.main()
