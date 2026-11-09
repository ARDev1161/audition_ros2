"""Fixture-level negative/positive checks with explicit failure-mode reporting."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1] / "benchmarks"))
from av_association_benchmark import benchmark, POSITIVE, HARD_NEGATIVES


class BenchmarkTests(unittest.TestCase):
    def test_reproducible_adversarial_confusion_matrix(self):
        run = benchmark(repetitions=20)
        self.assertEqual(run,benchmark(repetitions=20))
        self.assertEqual(run["total_trials"],240)
        # The unit-ray model *cannot* distinguish a co-linear impostor or
        # reflection; keep these failures explicit, rather than claiming 100%.
        self.assertEqual(run["temporal"]["tp"],40)
        self.assertEqual(run["temporal"]["fn"],0)
        self.assertEqual(run["temporal"]["fp"],40)
        self.assertEqual(run["temporal"]["tn"],160)
        self.assertEqual(run["instantaneous"]["fp"],80)
        self.assertEqual(run["instantaneous"]["tp"],40)
        for case in POSITIVE:
            self.assertEqual(run["per_case"][case]["temporally_confirmed"],20)
        for case in HARD_NEGATIVES:
            self.assertEqual(run["per_case"][case]["temporally_confirmed"],20)
        self.assertLess(run["temporal"]["false_positive_rate"],
                        run["instantaneous"]["false_positive_rate"])

    def test_small_parameterized_fixture(self):
        self.assertEqual(benchmark(repetitions=1)["total_trials"],12)
        with self.assertRaises(ValueError):
            benchmark(repetitions=0)


if __name__ == "__main__":
    unittest.main()
