"""M4 recording-byte integrity checks, using actual temporary PCM WAV files."""
import hashlib
import json
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest
import wave

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "benchmarks" / "evaluate_field_manifest.py"
EXAMPLE = PACKAGE / "test" / "fixtures" / "m4_field_contract_example.json"


class RecordingArtifactAuditTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = pathlib.Path(temp.name)
        self.capture_root = self.root / "captures"
        (self.capture_root / "nested").mkdir(parents=True)
        self.audio = self.capture_root / "nested" / "voice.wav"
        with wave.open(str(self.audio), "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(16000)
            wav.writeframes(struct.pack("<" + "h" * 1600, *([1200] * 1600)))
        self.digest = hashlib.sha256(self.audio.read_bytes()).hexdigest()
        self.data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        for trial in self.data["trials"]:
            trial["recording_sha256"] = self.digest
        self.manifest = self.root / "trials.json"
        self.manifest.write_text(json.dumps(self.data), encoding="utf-8")
        self.index = self.root / "recording_index.json"
        self.entries = [{"recording_sha256": self.digest,
                         "path": "nested/voice.wav"}]
        self.write_index()

    def write_index(self):
        self.index.write_text(json.dumps({"schema_version": 1,
                                          "recordings": self.entries}),
                              encoding="utf-8")

    def run_cli(self, include=True):
        command = [sys.executable, str(SCRIPT), str(self.manifest)]
        if include:
            command += ["--recording-index", str(self.index),
                        "--recordings-root", str(self.capture_root)]
        return subprocess.run(command, text=True, capture_output=True,
                              check=False)

    def test_matches_real_wav_bytes_and_preserves_no_physical_claim(self):
        run = self.run_cli()
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        audit = report["recording_artifact_audit"]
        self.assertTrue(audit["recording_bytes_verified"])
        self.assertEqual(audit["verified_unique_recordings"], 1)
        self.assertEqual(audit["verified_total_bytes"], self.audio.stat().st_size)
        self.assertEqual(audit["index_sha256"],
                         hashlib.sha256(self.index.read_bytes()).hexdigest())
        self.assertFalse(audit["origin_authenticated"])
        self.assertFalse(audit["physical_validation_claim"])
        self.assertFalse(report["safety_diagnostic"]["diagnostic_promotion_eligible"])
        self.assertFalse(report["safety_diagnostic"]["nav2_admission"])

    def test_missing_index_keeps_scoring_but_blocks_promotion(self):
        run = self.run_cli(include=False)
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        self.assertFalse(report["recording_artifact_audit"]["recording_bytes_verified"])
        self.assertIn("recording_bytes_not_verified",
                      report["safety_diagnostic"]["blockers"])

    def test_half_supplied_options_rejected(self):
        for option in ("--recording-index", "--recordings-root"):
            path = self.index if option.endswith("index") else self.capture_root
            run = subprocess.run([sys.executable, str(SCRIPT), str(self.manifest),
                                  option, str(path)], text=True, capture_output=True)
            self.assertEqual(run.returncode, 2)
            self.assertIn("both", run.stderr)

    def test_content_tampering_fails_closed(self):
        self.audio.write_bytes(self.audio.read_bytes() + b"tampered")
        run = self.run_cli()
        self.assertEqual(run.returncode, 2)
        self.assertIn("SHA-256 mismatch", run.stderr)
        self.assertFalse(run.stdout.strip())

    def test_missing_extra_duplicate_and_alias_index_rejected(self):
        second = self.capture_root / "nested" / "other.wav"
        second.write_bytes(b"separate recording")
        digest2 = hashlib.sha256(second.read_bytes()).hexdigest()
        variants = [
            [],
            self.entries + [{"recording_sha256": digest2, "path": "nested/other.wav"}],
            self.entries * 2,
            self.entries + [{"recording_sha256": digest2, "path": "nested/voice.wav"}],
        ]
        for entries in variants:
            with self.subTest(entries=entries):
                self.entries = entries
                self.write_index()
                self.assertEqual(self.run_cli().returncode, 2)

    def test_nested_paths_refuse_escape_symlinks_and_directories(self):
        outside = self.root / "outside.wav"
        outside.write_bytes(self.audio.read_bytes())
        (self.capture_root / "alias.wav").symlink_to(outside)
        (self.capture_root / "linked_dir").symlink_to(self.root, target_is_directory=True)
        variants = [
            "../outside.wav", "/tmp/outside.wav", "nested/../voice.wav",
            "nested//voice.wav", "nested/./voice.wav",
            r"nested\voice.wav", "alias.wav", "linked_dir/outside.wav",
            "nested",
        ]
        for value in variants:
            with self.subTest(path=value):
                self.entries = [{"recording_sha256": self.digest, "path": value}]
                self.write_index()
                run = self.run_cli()
                self.assertEqual(run.returncode, 2, (value, run.stderr))

    def test_duplicate_keys_bad_json_and_bad_digest_rejected(self):
        raw_variants = [
            '{"schema_version":1,"schema_version":1,"recordings":[]}',
            '{"schema_version":1,"recordings":NaN}',
            '{"schema_version":true,"recordings":[]}',
            '{"schema_version":1,"recordings":['
            '{"recording_sha256":"not-a-sha","path":"nested/voice.wav"}]}',
        ]
        for content in raw_variants:
            with self.subTest(content=content[:40]):
                self.index.write_text(content, encoding="utf-8")
                self.assertEqual(self.run_cli().returncode, 2)

    def test_two_distinct_recordings_are_both_checked(self):
        second = self.capture_root / "nested" / "other.wav"
        second.write_bytes(b"another recording payload")
        digest2 = hashlib.sha256(second.read_bytes()).hexdigest()
        new = dict(self.data["trials"][0])
        new["trial_id"] = "another-recording"
        new["recording_sha256"] = digest2
        self.data["trials"].append(new)
        self.manifest.write_text(json.dumps(self.data), encoding="utf-8")
        self.entries.append({"recording_sha256": digest2, "path": "nested/other.wav"})
        self.write_index()
        run = self.run_cli()
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        self.assertEqual(report["recording_artifact_audit"]["verified_unique_recordings"], 2)
        self.assertEqual(report["recording_artifact_audit"]["verified_total_bytes"],
                         self.audio.stat().st_size + second.stat().st_size)


if __name__ == "__main__":
    unittest.main()
