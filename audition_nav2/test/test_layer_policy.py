"""M5 integrated private costmap lifecycle tests without Nav2 dependencies."""
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nav_admission import MetricSource
from grid_projection import GridGeometry
from layer_policy import AcousticLayerPolicy

NOW = 2_000_000_000
COV = (.04, 0., 0., 0., .04, 0., 0., 0., .04)
GRID = GridGeometry(10, 10, 1., 0., 0.)

def metric(name='a', stamp=NOW-1, expires=NOW+100, valid=True):
    return MetricSource(name, 'sensor', stamp, expires, (2., 3., 0.),
                        COV, valid)

class LayerPolicyTests(unittest.TestCase):
    def test_requires_metric_validation_and_transform(self):
        policy = AcousticLayerPolicy()
        self.assertFalse(policy.ingest(metric(valid=False), NOW))
        self.assertTrue(policy.ingest(metric(), NOW))
        self.assertEqual(policy.tick(GRID, NOW)[0], {})
        self.assertEqual(policy.tick(GRID, NOW, transform=lambda s:(2.,3.))[0], {(2,3):100})

    def test_ttl_expires_without_new_messages(self):
        policy = AcousticLayerPolicy()
        policy.ingest(metric(), NOW)
        policy.tick(GRID, NOW, transform=lambda s:(2.,3.))
        self.assertEqual(policy.tick(GRID, NOW+100, transform=lambda s:(2.,3.))[:2], ({}, ((2,3),)))

    def test_missing_tf_fails_closed_and_revokes_private_cell(self):
        policy = AcousticLayerPolicy()
        policy.ingest(metric(), NOW)
        policy.tick(GRID, NOW, transform=lambda s:(2.,3.))
        def fail(_):
            raise LookupError('stale TF')
        self.assertEqual(policy.tick(GRID, NOW+1, transform=fail)[:2], ({}, ((2,3),)))

    def test_rolling_origin_invalidates_previous_geometry(self):
        policy = AcousticLayerPolicy()
        policy.ingest(metric(), NOW)
        policy.tick(GRID, NOW, transform=lambda s:(2.,3.))
        shifted = GridGeometry(10, 10, 1., 1., 0.)
        cells, removed, changed = policy.tick(shifted, NOW+1, transform=lambda s:(2.,3.))
        self.assertEqual(cells, {(1,3):100})
        self.assertEqual(removed, ((2,3),))
        self.assertTrue(changed)

    def test_multisource_and_explicit_revoke(self):
        policy = AcousticLayerPolicy()
        policy.ingest(metric('a'), NOW)
        policy.ingest(metric('b'), NOW)
        cells, _, _ = policy.tick(GRID, NOW, transform=lambda s:(2.,3.) if s.source_id=='a' else (4.,5.))
        self.assertEqual(cells, {(2,3):100,(4,5):100})
        policy.revoke('a')
        cells, removed, _ = policy.tick(GRID, NOW+1, transform=lambda s:(4.,5.))
        self.assertEqual(cells, {(4,5):100})
        self.assertEqual(removed, ((2,3),))

    def test_clock_rewind_retracts_even_if_old_source_is_eligible(self):
        policy = AcousticLayerPolicy()
        policy.ingest(metric(), NOW)
        policy.tick(GRID, NOW, transform=lambda s:(2.,3.))
        self.assertEqual(policy.tick(GRID, NOW-1, transform=lambda s:(2.,3.))[:2], ({}, ((2,3),)))

    def test_capacity_overflow_rejected(self):
        policy = AcousticLayerPolicy(capacity=1)
        self.assertTrue(policy.ingest(metric('a'), NOW))
        self.assertFalse(policy.ingest(metric('b'), NOW))

    def test_nonfinite_transform_retracts_all_sources(self):
        policy = AcousticLayerPolicy()
        self.assertTrue(policy.ingest(metric('a'), NOW))
        self.assertTrue(policy.ingest(metric('b'), NOW))
        policy.tick(GRID, NOW, transform=lambda s: (2., 3.) if s.source_id == 'a' else (4., 5.))
        cells, removed, _ = policy.tick(
            GRID, NOW+1,
            transform=lambda s: (2., 3.) if s.source_id == 'a' else (float('nan'), 5.))
        self.assertEqual(cells, {})
        self.assertEqual(removed, ((2,3), (4,5)))

    def test_outside_map_finite_source_is_not_a_failure(self):
        policy = AcousticLayerPolicy()
        policy.ingest(metric('a'), NOW)
        policy.ingest(metric('b'), NOW)
        cells, _, _ = policy.tick(
            GRID, NOW, transform=lambda s: (2.,3.) if s.source_id == 'a' else (100.,100.))
        self.assertEqual(cells, {(2,3):100})

    def test_invalid_cost_never_lethal(self):
        with self.assertRaises(ValueError):
            AcousticLayerPolicy(maximum_cost=253)

if __name__ == '__main__':
    unittest.main()
