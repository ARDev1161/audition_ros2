"""M5 fail-closed soft-cost cell lifecycle tests (no ROS hardware)."""
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nav_admission import MetricSource
from soft_cost_cells import SoftCostCells

T = 2_000_000_000
COV = (.04, 0., 0., 0., .04, 0., 0., 0., .04)

def source(name='s', expiry=T+100, validated=True):
    return MetricSource(name, 'map', T-10, expiry, (2., 3., 0.), COV, validated)

class SoftCostCellsTests(unittest.TestCase):
    def test_no_tf_never_paints_and_clears_old_cells(self):
        cells = SoftCostCells(resolution_m=1)
        self.assertEqual(cells.reconcile([source()], T), ({}, ()))
        self.assertEqual(cells.reconcile([source()], T, transform=lambda s:(2.,3.)), ({(2,3):100}, ()))
        self.assertEqual(cells.reconcile([source()], T), ({}, ((2,3),)))

    def test_expiry_silence_and_replacement_clear(self):
        cells = SoftCostCells(resolution_m=1)
        tf = lambda s: (2.,3.)
        cells.reconcile([source()], T, transform=tf)
        self.assertEqual(cells.reconcile([], T+1, transform=tf), ({}, ((2,3),)))
        # Keep clock monotonic: a rewind is tested separately below.
        cells.reconcile([source()], T+1, transform=tf)
        self.assertEqual(cells.reconcile([source()], T+100, transform=tf), ({}, ((2,3),)))

    def test_reject_unvalidated_and_diagnostic_objects(self):
        cells = SoftCostCells()
        self.assertEqual(cells.reconcile([source(validated=False), object()], T, transform=lambda s:(2.,3.)), ({}, ()))

    def test_invalid_transform_clears_instead_of_stale_cost(self):
        cells = SoftCostCells(resolution_m=1)
        cells.reconcile([source()], T, transform=lambda s:(2.,3.))
        def missing_tf(s):
            raise LookupError('missing stamped TF')
        self.assertEqual(cells.reconcile([source()], T, transform=missing_tf), ({}, ((2,3),)))

    def test_budget_overflow_fails_closed(self):
        cells = SoftCostCells(resolution_m=1, max_cells=1)
        self.assertEqual(cells.reconcile([source('a'),source('b')], T, transform=lambda s:(1.,1.) if s.source_id=='a' else (2.,2.)), ({}, ()))

    def test_clock_rewind_clears_even_when_source_appears_valid(self):
        cells = SoftCostCells(resolution_m=1)
        tf = lambda s: (2.,3.)
        cells.reconcile([source()], T+50, transform=tf)
        # An older clock epoch must never preserve previously painted cells.
        self.assertEqual(cells.reconcile([source()], T, transform=tf), ({}, ((2,3),)))
        self.assertEqual(cells.reconcile([], T+1, transform=tf), ({}, ()))

    def test_bad_clock_revokes_all_owned_cells(self):
        cells = SoftCostCells(resolution_m=1)
        cells.reconcile([source()], T, transform=lambda s:(2.,3.))
        self.assertEqual(cells.reconcile([source()], 0, transform=lambda s:(2.,3.)), ({}, ((2,3),)))

    def test_same_cell_combines_nonlethal(self):
        cells = SoftCostCells(resolution_m=1, maximum_cost=252)
        self.assertEqual(cells.reconcile([source('a'),source('b')], T, transform=lambda s:(1.,1.)), ({(1,1):252}, ()))

    def test_source_budget_rejects_same_cell_flood_and_revokes_old_overlay(self):
        # A cell budget alone cannot bound CPU: thousands of distinct source
        # IDs can all map to the same cell, causing thousands of TF lookups.
        cells = SoftCostCells(resolution_m=1, max_cells=1, max_sources=3)
        calls = []
        def tf(s):
            calls.append(s.source_id)
            return (2., 3.)
        self.assertEqual(cells.reconcile([source('baseline')], T, transform=tf),
                         ({(2, 3): 100}, ()))
        calls.clear()
        flooded = (source('flood-%d' % i) for i in range(1_000_000))
        self.assertEqual(cells.reconcile(flooded, T+1, transform=tf),
                         ({}, ((2, 3),)))
        self.assertEqual(calls, ['flood-0', 'flood-1', 'flood-2'])
        self.assertEqual(cells.reconcile([], T+2, transform=tf), ({}, ()))

    def test_source_budget_includes_invalid_evidence(self):
        # Untrusted/invalid entries also consume the bounded ingress budget.
        cells = SoftCostCells(resolution_m=1, max_sources=2)
        calls = []
        def tf(s):
            calls.append(s.source_id)
            return (2., 3.)
        self.assertEqual(cells.reconcile(
            [object(), source('good'), source('overflow')], T,
            transform=tf), ({}, ()))
        self.assertEqual(calls, ['good'])

    def test_source_budget_exact_boundary_and_recovery(self):
        cells = SoftCostCells(resolution_m=1, max_sources=2)
        tf = lambda s: (2., 3.)
        self.assertEqual(cells.reconcile(
            [source('a'), source('b')], T, transform=tf),
            ({(2, 3): 100}, ()))
        self.assertEqual(cells.reconcile(
            [source('a'), source('b'), source('c')], T+1,
            transform=tf), ({}, ((2, 3),)))
        self.assertEqual(cells.reconcile(
            [source('a')], T+2, transform=tf), ({(2, 3): 100}, ()))

    def test_source_budget_rejects_invalid_configuration(self):
        for value in (0, -1, 65537, 1.0, True, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                SoftCostCells(max_sources=value)

if __name__ == '__main__':
    unittest.main()
