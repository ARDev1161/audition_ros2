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
        cells.reconcile([source()], T, transform=tf)
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

    def test_same_cell_combines_nonlethal(self):
        cells = SoftCostCells(resolution_m=1, maximum_cost=252)
        self.assertEqual(cells.reconcile([source('a'),source('b')], T, transform=lambda s:(1.,1.)), ({(1,1):252}, ()))

if __name__ == '__main__':
    unittest.main()
