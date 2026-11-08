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
         "session_id": "session-" + t["recording_sha256"][-12:],
         "scene_id": "scene-" + t["recording_sha256"][:12],
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

    def test_false_pair_scene_counts_are_derived_from_validated_snapshot(self):
        source = self.dataset["trials"][0]
        source["predictions"]["temporal_fusion"] = {
            "decision": "pair", "pair": {
                "audio_id": "wrong-audio", "radar_id": "wrong-radar"}}
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        verified = audit_acquisition_provenance(self.audit_file, self.dataset)
        self.assertEqual(verified["held_out_false_pair_scene_counts"].get(source["stratum"]), 1)
        # A later edit to the path cannot change the previously computed audit.
        self.audit_file.write_text('{"trials":[]}', encoding="utf-8")
        self.assertEqual(verified["held_out_false_pair_scene_counts"].get(source["stratum"]), 1)
        with self.assertRaises(ValueError):
            audit_acquisition_provenance(self.audit_file, self.dataset)

    def test_scene_truth_class_fraction_requires_both_classes(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--acquisition-provenance", str(self.audit_file),
               "--required-stratum", "reflection",
               "--maximum-false-pair-scene-fraction-per-truth-class", "1",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        classes = report["required_strata_false_pair_scene_fraction_by_truth_class"]["reflection"]
        self.assertIn("positive", classes)
        self.assertIn("negative", classes)
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_scene_truth_class_fraction_rejects_missing_provenance(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "direct",
               "--maximum-false-pair-scene-fraction-per-truth-class", "0.5",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        self.assertIn("required_false_pair_scene_classes_unverifiable",
                      json.loads(result.stdout)["safety_diagnostic"]["blockers"])

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

    def test_unverified_domain_blocks_promotion_even_with_acquisition_audit(self):
        cli = self.cli()
        self.assertEqual(cli.returncode, 0, cli.stderr)
        report = json.loads(cli.stdout)
        self.assertIn("data_domain_unverified", report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["no_observed_safety_regression"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_simulation_lineage_cannot_be_laundered_through_acquisition_audit(self):
        manifest_raw = self.manifest.read_bytes()
        sidecar = self.manifest.with_name(self.manifest.name + ".provenance.json")
        sidecar.write_text(json.dumps({
            "domain": "simulation", "physical_validation_claim": False,
            "simulation_export_sha256": "f" * 64,
            "manifest_sha256": hashlib.sha256(manifest_raw).hexdigest(),
            "trials": len(self.dataset["trials"]),
        }), encoding="utf-8")
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--require-simulation-provenance",
               "--acquisition-provenance", str(self.audit_file)]
        cli = subprocess.run(cmd, text=True, capture_output=True, check=False)
        self.assertEqual(cli.returncode, 0, cli.stderr)
        report = json.loads(cli.stdout)
        self.assertEqual(report["data_domain"], "simulation")
        self.assertTrue(report["acquisition_provenance_audit"]["crosscheck_passed"])
        self.assertIn("simulation_only_evidence", report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["no_observed_safety_regression"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertFalse(report["safety_diagnostic"]["physical_localization_accepted"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_required_adverse_strata_fail_closed_without_observable_truth(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "reflection",
               "--required-stratum", "not-in-dataset",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["required_strata"], ["not-in-dataset", "reflection"])
        self.assertIn("required_stratum_missing_or_unobservable:not-in-dataset",
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_all_splits_cannot_launder_development_only_adverse_coverage(self):
        # Even with --all-splits, required adverse conditions need held-out
        # observable truth, not a development-only sample.
        source = dict(self.dataset["trials"][0])
        source["trial_id"] = "development-only-reflection"
        source["stratum"] = "reflection"
        source["split"] = "development"
        source["recording_sha256"] = "e" * 64
        self.dataset["trials"].append(source)
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--all-splits", "--required-stratum", "reflection",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertIn("required_stratum_missing_or_unobservable:reflection",
                      report["safety_diagnostic"]["blockers"])
        self.assertEqual(report["required_strata_held_out_coverage"]["reflection"],
                         {"observable_positive": 0, "observable_negative": 0,
                          "unobservable": 0})

    def test_required_stratum_with_unknown_truth_only_is_not_covered(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "occluded",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertIn("required_stratum_missing_or_unobservable:occluded",
                      report["safety_diagnostic"]["blockers"])
        self.assertEqual(report["required_strata_held_out_coverage"]["occluded"]["unobservable"], 1)

    def test_required_stratum_needs_both_observable_truth_classes(self):
        sample = dict(self.dataset["trials"][0])
        sample["trial_id"] = "negative-only-stratum"
        sample["stratum"] = "negative-only"
        sample["split"] = "held_out"
        sample["truth"] = dict(sample["truth"], observable=True, pair=None)
        self.dataset["trials"].append(sample)
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "negative-only",
               "--require-both-truth-classes", "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertIn("required_stratum_missing_truth_class:negative-only",
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_both_classes_requires_explicit_stratum(self):
        result = subprocess.run([sys.executable, str(SCRIPT), str(self.manifest),
                                 "--require-both-truth-classes"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires --required-stratum", result.stderr)

    def test_invalid_required_stratum_is_input_error(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", " bad ", "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("invalid required stratum name", result.stderr)

    def test_repeated_trials_do_not_inflate_distinct_recording_coverage(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "reflection",
               "--minimum-recordings-per-truth-class", "2",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        counts = report["required_strata_recording_coverage"]["reflection"]
        self.assertLess(counts["positive"], 2)
        self.assertIn("required_stratum_insufficient_recordings:reflection:positive",
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_recording_coverage_flag_requires_named_strata(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--minimum-recordings-per-truth-class", "2"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires --required-stratum", result.stderr)

    def test_acquisition_truth_class_groups_count_unique_claimed_ids(self):
        audit = audit_acquisition_provenance(self.audit_file, self.dataset)
        groups = audit["held_out_truth_class_groups"]
        self.assertIn("direct", groups)
        self.assertIn("positive", groups["direct"])
        self.assertGreaterEqual(groups["direct"]["positive"]["session_id"], 1)
        # Many trials from the same capture are one scene, not independent scenes.
        self.assertEqual(groups["direct"]["positive"]["scene_id"], 1)

    def test_required_acquisition_groups_block_single_scene(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--acquisition-provenance", str(self.audit_file),
               "--required-stratum", "direct",
               "--minimum-acquisition-groups-per-truth-class", "2",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["required_strata_acquisition_group_coverage"]["direct"]["positive"]["scene_id"], 1)
        self.assertIn("required_stratum_insufficient_acquisition_groups:direct:positive:scene_id",
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_acquisition_group_gate_requires_provenance(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "direct",
               "--minimum-acquisition-groups-per-truth-class", "2",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        self.assertIn("required_acquisition_groups_unverifiable",
                      json.loads(result.stdout)["safety_diagnostic"]["blockers"])

    def test_false_pair_recording_gate_counts_unique_held_out_files(self):
        # Explicitly inject two wrong pairs from the same immutable recording.
        rows = [row for row in self.dataset["trials"]
                if row["split"] == "held_out" and row["truth"]["observable"]]
        source = rows[0]
        wrong = {"decision": "pair", "pair": {
            "audio_id": "incorrect-audio", "radar_id": "incorrect-radar"}}
        for i in range(2):
            sample = json.loads(json.dumps(source))
            sample["trial_id"] = "false-pair-replay-" + str(i)
            sample["stratum"] = "adverse-replay"
            sample["predictions"]["temporal_fusion"] = wrong
            self.dataset["trials"].append(sample)
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        base = [sys.executable, str(SCRIPT), str(self.manifest),
                "--required-stratum", "adverse-replay",
                "--maximum-false-pair-recordings-per-stratum"]
        blocked = subprocess.run(base + ["0", "--require-diagnostic-eligibility"],
                                 capture_output=True, text=True)
        self.assertEqual(blocked.returncode, 3, blocked.stderr)
        report = json.loads(blocked.stdout)
        self.assertEqual(report["required_strata_false_pair_recordings"]["adverse-replay"], 1)
        self.assertIn("required_stratum_false_pair_recording_limit:adverse-replay",
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])
        permitted = subprocess.run(base + ["1"], capture_output=True, text=True)
        self.assertEqual(permitted.returncode, 0, permitted.stderr)
        self.assertNotIn("required_stratum_false_pair_recording_limit:adverse-replay",
                         json.loads(permitted.stdout)["safety_diagnostic"]["blockers"])

    def test_false_pair_limit_rejects_invalid_configuration(self):
        for args in (["--maximum-false-pair-recordings-per-stratum", "0"],
                     ["--required-stratum", "direct",
                      "--maximum-false-pair-recordings-per-stratum", "-1"]):
            result = subprocess.run([sys.executable, str(SCRIPT), str(self.manifest)] + args,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)

    def test_false_pair_recording_fraction_uses_unique_held_out_denominator(self):
        source = next(row for row in self.dataset["trials"]
                      if row["split"] == "held_out" and row["truth"]["observable"])
        for i in range(3):
            sample = json.loads(json.dumps(source))
            sample["trial_id"] = "fraction-repeat-" + str(i)
            sample["stratum"] = "fraction-test"
            sample["predictions"]["temporal_fusion"] = {
                "decision": "pair", "pair": {
                    "audio_id": "wrong", "radar_id": "wrong"}}
            self.dataset["trials"].append(sample)
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        base = [sys.executable, str(SCRIPT), str(self.manifest),
                "--required-stratum", "fraction-test",
                "--maximum-false-pair-recording-fraction"]
        blocked = subprocess.run(base + ["0.5", "--require-diagnostic-eligibility"],
                                 capture_output=True, text=True)
        self.assertEqual(blocked.returncode, 3, blocked.stderr)
        report = json.loads(blocked.stdout)
        counts = report["required_strata_false_pair_recording_fraction"]["fraction-test"]
        self.assertEqual(counts, {"observable_recordings": 1,
                                  "false_pair_recordings": 1, "fraction": 1.0})
        self.assertIn("required_stratum_false_pair_recording_fraction_limit:fraction-test",
                      report["safety_diagnostic"]["blockers"])
        allowed = subprocess.run(base + ["1"], capture_output=True, text=True)
        self.assertEqual(allowed.returncode, 0, allowed.stderr)
        self.assertNotIn("required_stratum_false_pair_recording_fraction_limit:fraction-test",
                         json.loads(allowed.stdout)["safety_diagnostic"]["blockers"])

    def test_false_pair_fraction_rejects_invalid_bounds_and_missing_strata(self):
        for args in (["--maximum-false-pair-recording-fraction", "0.5"],
                     ["--required-stratum", "direct",
                      "--maximum-false-pair-recording-fraction", "nan"],
                     ["--required-stratum", "direct",
                      "--maximum-false-pair-recording-fraction", "1.1"]):
            result = subprocess.run([sys.executable, str(SCRIPT), str(self.manifest)] + args,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)

    def test_false_pair_truth_class_fraction_cannot_hide_negative_failures(self):
        # A zero denominator must block rather than be reported as zero errors.
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "reflection",
               "--maximum-false-pair-fraction-per-truth-class", "1",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        classes = report["required_strata_false_pair_fraction_by_truth_class"]["reflection"]
        for kind in ("positive", "negative"):
            self.assertIn("observable_recordings", classes[kind])
            self.assertIn("false_pair_recordings", classes[kind])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_false_pair_truth_class_fraction_rejects_bad_configuration(self):
        for args in (["--maximum-false-pair-fraction-per-truth-class", "0"],
                     ["--required-stratum", "direct",
                      "--maximum-false-pair-fraction-per-truth-class", "nan"],
                     ["--required-stratum", "direct",
                      "--maximum-false-pair-fraction-per-truth-class", "1.01"]):
            result = subprocess.run([sys.executable, str(SCRIPT), str(self.manifest)] + args,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)

    def test_false_pair_scene_gate_rejects_missing_acquisition(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "direct",
               "--maximum-false-pair-scenes-per-stratum", "0",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        self.assertIn("required_false_pair_scenes_unverifiable",
                      json.loads(result.stdout)["safety_diagnostic"]["blockers"])

    def test_false_pair_scene_gate_counts_distinct_acquisition_scenes(self):
        source = next(row for row in self.dataset["trials"]
                      if row["split"] == "held_out" and row["truth"]["observable"])
        source["predictions"]["temporal_fusion"] = {
            "decision": "pair", "pair": {
                "audio_id": "wrong-audio", "radar_id": "wrong-radar"}}
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--acquisition-provenance", str(self.audit_file),
               "--required-stratum", source["stratum"],
               "--maximum-false-pair-scenes-per-stratum", "0",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertGreaterEqual(report["required_strata_false_pair_scene_coverage"][source["stratum"]], 1)
        self.assertIn("required_stratum_false_pair_scene_limit:" + source["stratum"],
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_recording_gain_requires_actual_held_out_advantage(self):
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--required-stratum", "direct", "--require-held-out-recording-gain",
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        gain = report["required_strata_recording_gain"]["direct"]
        for baseline in ("audio_only", "radar_only", "instant_fusion"):
            self.assertEqual(set(gain[baseline]), {"helped", "harmed", "tied"})
            self.assertEqual(sum(gain[baseline].values()),
                             len({row["recording_sha256"] for row in self.dataset["trials"]
                                  if row["stratum"] == "direct" and row["split"] == "held_out"
                                  and row["truth"]["observable"]}))
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_recording_gain_requires_explicit_stratum(self):
        result = subprocess.run([sys.executable, str(SCRIPT), str(self.manifest),
                                 "--require-held-out-recording-gain"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)

    def test_strict_cli_exit_code_blocks_unverified_field_evidence(self):
        # Default scoring is useful for diagnostics, but automation requiring
        # eligible evidence must not mistake a JSON report for acceptance.
        base = [sys.executable, str(SCRIPT), str(self.manifest),
                "--acquisition-provenance", str(self.audit_file)]
        scored = subprocess.run(base, capture_output=True, text=True)
        self.assertEqual(scored.returncode, 0, scored.stderr)
        strict = subprocess.run(base + ["--require-diagnostic-eligibility"],
                                capture_output=True, text=True)
        self.assertEqual(strict.returncode, 3, strict.stderr)
        report = json.loads(strict.stdout)
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])
        self.assertIn("data_domain_unverified", strict.stderr)

    def test_strict_cli_malformed_input_still_returns_validation_error(self):
        self.audit_file.write_text('{"trials":[]}', encoding="utf-8")
        cmd = [sys.executable, str(SCRIPT), str(self.manifest),
               "--acquisition-provenance", str(self.audit_file),
               "--require-diagnostic-eligibility"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("Invalid M4 evaluation input:", result.stderr)

    def test_missing_audit_downgrades_diagnostic_not_scoring(self):
        cli = self.cli(supply=False)
        self.assertEqual(cli.returncode, 0, cli.stderr)
        report = json.loads(cli.stdout)
        self.assertEqual(report["acquisition_provenance_audit"]["status"], "not_provided")
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertIn("acquisition_provenance_not_supplied",
                      report["safety_diagnostic"]["blockers"])
        self.assertFalse(report["safety_diagnostic"]["no_observed_safety_regression"])

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

    def test_same_recording_with_conflicting_session_id_rejected(self):
        # The fixture has several events from exactly the same recording;
        # giving one event a new session label must be rejected.
        self.audit["trials"][1]["session_id"] = "fake-other-session"
        self.write_audit()
        cli = self.cli()
        self.assertEqual(cli.returncode, 2)
        self.assertIn("provenance_recording_session_conflict", cli.stderr)

    def test_ambiguous_acquisition_identifiers_rejected(self):
        # Invisible controls, whitespace aliases and overlong labels must
        # never inflate the number of supposedly independent acquisitions.
        for field in ("trial_id", "session_id", "scene_id", "subject_id"):
            for bad in ("room\\n1", " room", "room ", "équipe", "a" * 129):
                with self.subTest(field=field, bad=bad):
                    self.audit = acquisition_rows(self.dataset)
                    self.audit["trials"][0][field] = bad
                    self.write_audit()
                    result = self.cli()
                    self.assertEqual(result.returncode, 2, result.stderr)

    def test_same_recording_with_conflicting_scene_id_rejected(self):
        # A recording cannot be relabeled into independent rooms by trial.
        self.audit["trials"][1]["scene_id"] = "fake-second-room"
        self.write_audit()
        cli = self.cli()
        self.assertEqual(cli.returncode, 2)
        self.assertIn("provenance_recording_scene_conflict", cli.stderr)

    def test_subject_changes_within_recording_are_allowed(self):
        # Multiple real people may legitimately appear in one capture.
        self.audit["trials"][1]["subject_id"] = "different-person"
        self.write_audit()
        self.assertEqual(self.cli().returncode, 0)

    def test_five_files_from_one_scene_are_not_five_independent_scenes(self):
        source = self.dataset["trials"][0]
        self.dataset["trials"] = [
            dict(source, trial_id="independent-" + str(i),
                 recording_sha256=format(i + 1, "064x"))
            for i in range(5)
        ]
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        self.audit = acquisition_rows(self.dataset)
        for row in self.audit["trials"]:
            row["scene_id"] = "shared-room"
        self.write_audit()
        cli = self.cli()
        self.assertEqual(cli.returncode, 0, cli.stderr)
        result = json.loads(cli.stdout)
        summary = result["acquisition_provenance_audit"]["independence_summary"]
        self.assertEqual(summary["overall_distinct_groups"]["session_id"], 5)
        self.assertEqual(summary["overall_distinct_groups"]["scene_id"], 1)
        self.assertEqual(summary["per_stratum"]["direct"]["distinct_groups"]["scene_id"], 1)
        self.assertIn(
            "insufficient_acquisition_groups:direct:scene_id",
            result["safety_diagnostic"]["blockers"])
        self.assertFalse(result["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertFalse(result["safety_diagnostic"]["physical_localization_accepted"])

    def test_unobservable_trials_are_not_independent_evidence(self):
        for trial in self.dataset["trials"]:
            trial["truth"] = {"observable": False, "pair": None}
        self.manifest.write_text(json.dumps(self.dataset), encoding="utf-8")
        cli = self.cli()
        self.assertEqual(cli.returncode, 0, cli.stderr)
        summary = json.loads(cli.stdout)["acquisition_provenance_audit"]["independence_summary"]
        self.assertEqual(summary["observable_trials"], 0)
        self.assertEqual(summary["overall_distinct_groups"]["session_id"], 0)
        self.assertEqual(summary["unknown_truth_excluded"], len(self.dataset["trials"]))

    def test_nonstring_group_fields_fail_closed_without_traceback(self):
        # JSON arrays/objects must never reach dictionary setdefault keys.
        for field in ("split", "recording_sha256", "session_id",
                      "scene_id", "subject_id"):
            for bad in ([], {"unexpected": "object"}):
                with self.subTest(field=field, bad=bad):
                    self.audit = acquisition_rows(self.dataset)
                    self.audit["trials"][0][field] = bad
                    self.write_audit()
                    result = self.cli()
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn("Invalid M4 evaluation input:", result.stderr)
                    self.assertNotIn("Traceback", result.stderr)

    def test_unhashable_trial_ids_fail_closed(self):
        self.audit["trials"][0]["trial_id"] = ["invalid"]
        self.write_audit()
        cli = self.cli()
        self.assertEqual(cli.returncode, 2)
        self.assertIn("provenance_trial_duplicate_or_invalid", cli.stderr)


if __name__ == "__main__":
    unittest.main()
