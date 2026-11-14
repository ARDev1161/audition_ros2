"""M5 admission tests: all unvalidated or stale evidence must fail closed."""
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nav_admission import MetricSource, eligible, soft_cost

T = 2_000_000_000
COV = (.04, 0., 0., 0., .04, 0., 0., 0., .04)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.source = MetricSource('speaker-1', 'map', T-100, T+100,
                                   (2., 3., 0.), COV, True)

    def test_only_validated_metric_source_admitted(self):
        self.assertTrue(eligible(self.source, T))
        self.assertEqual(soft_cost(self.source, T), 100)
        self.assertIsNone(soft_cost(replace(self.source, physically_validated=False), T))
        self.assertIsNone(soft_cost(object(), T))

    def test_expiry_and_clock(self):
        for now in (0, T+100, T+1000, T-101):
            self.assertFalse(eligible(self.source, now))
        self.assertFalse(eligible(replace(self.source, stamp_ns=T+1), T))
        self.assertFalse(eligible(replace(self.source, expires_ns=T-100), T))

    def test_invalid_frame_and_geometry(self):
        for changed in (
            {'frame_id': '/map'}, {'frame_id': ''},
            {'xyz_m': (float('nan'), 0., 0.)},
            {'covariance_xyz': (-1., 0., 0., 0., .04, 0., 0., 0., .04)},
            {'covariance_xyz': (.04, .2, 0., .2, .04, 0., 0., 0., .04)},
            {'covariance_xyz': (.04, 0., 0., .3, .04, 0., 0., 0., .04)},
        ):
            self.assertFalse(eligible(replace(self.source, **changed), T))

    def test_no_lethal_cost(self):
        for cost in (0, 253, 254, 255):
            with self.assertRaises(ValueError):
                soft_cost(self.source, T, maximum_cost=cost)
        self.assertEqual(soft_cost(self.source, T, maximum_cost=252), 252)


if __name__ == '__main__':
    unittest.main()
