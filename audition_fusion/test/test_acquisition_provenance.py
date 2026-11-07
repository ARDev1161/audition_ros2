"""Acquisition provenance must be complete, matched, and split-isolated."""
import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from acquisition_provenance import audit_acquisition_provenance

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "benchmarks" / "evaluate_field_manifest.py"
EXAMPLE = PACKAGE / "test" / "fixtures" / "m4_field_contract_example.json"


def acquisition_rows(dataset):
    return {"trials": [
        {"trial_id": t["trial_id"], "split": t["split"],
         "recording_sha256": t["recording_sha256"],
         "session_id": "session-" + str(i),
         "scene_id": "scene-" + str(i),
         "subject_id": "subject-" + str(i)}
        for i, t in enumerate(dataset["trials"])
    ]}


class AcquisitionProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name)
        self.dataset = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        self.manifest = self.root / "trials.json"
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        self.audit_file = self.root / "acquisition.json"
        self.audit = acquisition_rows(self.dataset)
        self.write_audit()

    def write_audit(self):
        self.audit_file.write_text(json.dumps(self.audit), encoding="utf-8")

    def cli(self, *, supply=True):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest)]
        if supply:
            cmd += ["--acquisition-provenance", str(self.audit_file)]
        return subprocess.run(cmd, text=True, capture_output=True, check=False)

    def test_complete_audit_crosschecked_but_never_physically_validated(self):
        result = audit_acquisition_provenance(self.audit_file, self.dataset)
        self.assertTrue(result["crosscheck_passed"])
        self.assertEqual(result["trial_count"], len(self.dataset["trials"]))
        self.assertEqual(result["provenance_sha256"],
                         hashlib.sha256(self.audit_file.read_bytes()).hexdigest())
        self.assertFalse(result["independently_authenticated"])
        cli = self.cli()
        self.assertEqual(cli.returncode, 0, cli.stderr)
        report = json.loads(cli.stdout)
        self.assertTrue(report["acquisition_provenance_audit"]["crosscheck_passed"])
        self.assertFalse(report["physical_validation_claim"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_missing_audit_downgrades_diagnostic_not_scoring(self):
        cli = self.cli(supply=False)
        self.assertEqual(cli.returncode, 0, cli.stderr)
        report = json.loads(cli.stdout)
        self.assertEqual(report["acquisition_provenance_audit"]["status"], "not_provided")
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertIn("acquisition_provenance_not_supplied",
                      report["safety_diagnostic"]["blockers"])

    def test_missing_extra_or_mutated_trial_rejected(self):
        variants = [
            self.audit["trials"][1:],
            self.audit["trials"] + [dict(self.audit["trials"][0], trial_id="extra")],
            [dict(row, split=("development" if i == 0 else row["split"]))
             for i, row in enumerate(self.audit["trials"])],
            [dict(row, recording_sha256=("a" * 64 if i == 0 else row["recording_sha256"]))
             for i, row in enumerate(self.audit["trials"])],
        ]
        for rows in variants:
            with self.subTest(rows=rows):
                self.audit = {"trials": rows}
                self.write_audit()
                self.assertEqual(self.cli().returncode, 2)

    def test_session_scene_subject_leakage_rejected(self):
        # Separate immutable recordings may still share a room/session/person
        # across held-out/development; no such leakage is acceptable.
        self.dataset["trials"][0]["split"] = "development"
        self.dataset["trials"][0]["recording_sha256"] = "e" * 64
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        for field in ("session_id", "scene_id", "subject_id"):
            self.audit = acquisition_rows(self.dataset)
            self.audit["trials"][1][field] = self.audit["trials"][0][field]
            self.write_audit()
            cli = self.cli()
            self.assertEqual(cli.returncode, 2, (field, cli.stderr))
            self.assertIn("provenance_split_leak:" + field, cli.stderr)

    def test_duplicates_nonfinite_malformed_and_oversized_rejected(self):
        for raw in ('{"trials":[],"trials":[]}', '{"trials": NaN}',
                    '{"trials": [{}]}', " " * (4 * 1024 * 1024 + 1)):
            with self.subTest(size=len(raw)):
                self.audit_file.write_text(raw, encoding="utf-8")
                self.assertEqual(self.cli().returncode, 2)

    def test_unhashable_trial_ids_fail_closed(self):
        self.audit["trials"][0]["trial_id"] = ["invalid"]
        self.write_audit()
        cli = self.cli()
        self.assertEqual(cli.returncode, 2)
        self.assertIn("provenance_trial_duplicate_or_invalid", cli.stderr)


if __name__ == "__main__":
    unittest.main()
