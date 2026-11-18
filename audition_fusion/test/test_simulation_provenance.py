"""Provenance verification must fail closed on tampering and fake origin."""
import hashlib
import json
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from simulation_provenance import verified_simulation_provenance


class SimulationProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = pathlib.Path(self.tmp.name) / "manifest.json"
        self.manifest = b'{"schema_version":1,"trials":[{},{}]}\n'
        self.path.write_bytes(self.manifest)
        self.data = {"trials": [{}, {}]}
        self.provenance = {
            "domain": "simulation", "physical_validation_claim": False,
            "simulation_export_sha256": "a" * 64,
            "manifest_sha256": hashlib.sha256(self.manifest).hexdigest(),
            "trials": 2,
        }
        self.sidecar = self.path.with_name(self.path.name + ".provenance.json")
        self.write_sidecar()

    def write_sidecar(self):
        self.sidecar.write_text(json.dumps(self.provenance), encoding="utf-8")

    def verify(self):
        return verified_simulation_provenance(
            self.path, self.path.read_bytes(), self.data)

    def test_valid_simulation(self):
        report = self.verify()
        self.assertTrue(report["provenance_verified"])
        self.assertEqual(report["data_domain"], "simulation")
        self.assertEqual(report["simulation_export_sha256"], "a" * 64)

    def test_missing_sidecar(self):
        self.sidecar.unlink()
        with self.assertRaises(FileNotFoundError):
            self.verify()

    def test_tampered_manifest(self):
        self.path.write_bytes(self.manifest + b" ")
        with self.assertRaisesRegex(ValueError, "digest mismatch"):
            self.verify()

    def test_tampered_metadata(self):
        for key, value, message in [
            ("domain", "physical", "physical claims"),
            ("physical_validation_claim", True, "physical claims"),
            ("trials", True, "trial count"),
            ("trials", 3, "trial count"),
            ("simulation_export_sha256", "invalid", "digest"),
        ]:
            with self.subTest(key=key, value=value):
                original = self.provenance[key]
                try:
                    self.provenance[key] = value
                    self.write_sidecar()
                    with self.assertRaisesRegex(ValueError, message):
                        self.verify()
                finally:
                    self.provenance[key] = original
                    self.write_sidecar()

    def test_unknown_fields_and_duplicate_keys(self):
        self.sidecar.write_text('{"domain":"simulation","domain":"physical"}')
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.verify()
        self.provenance["attested"] = True
        self.write_sidecar()
        with self.assertRaisesRegex(ValueError, "schema"):
            self.verify()

    def test_nonfinite_and_oversized_sidecar(self):
        self.sidecar.write_text('{"trials": NaN}')
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.verify()
        self.sidecar.write_bytes(b" " * (64 * 1024 + 1))
        with self.assertRaisesRegex(ValueError, "too large"):
            self.verify()


if __name__ == "__main__":
    unittest.main()
