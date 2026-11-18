"""Synthetic static/rolling/resized Nav2 grid projection acceptance tests."""
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from grid_projection import AcousticGridProjection, GridGeometry

G = GridGeometry(10, 10, 1., 0., 0.)

class GridProjectionTests(unittest.TestCase):
    def test_static_add_remove_and_overlap(self):
        p = AcousticGridProjection()
        self.assertEqual(p.reconcile(G, [(2.5,3.5,40),(2.6,3.4,90)]),
                         ({(2,3):90}, (), True))
        self.assertEqual(p.reconcile(G, [(4.,4.,50)]),
                         ({(4,4):50}, ((2,3),), False))
        self.assertEqual(p.reconcile(G, []), ({}, ((4,4),), False))

    def test_rolling_origin_invalidates_old_indices(self):
        p = AcousticGridProjection()
        p.reconcile(G, [(2.5,3.5,90)])
        shifted = GridGeometry(10, 10, 1., 1., 0.)
        self.assertEqual(p.reconcile(shifted, [(2.5,3.5,90)]),
                         ({(1,3):90}, ((2,3),), True))

    def test_resize_invalidates_even_if_cell_index_same(self):
        p = AcousticGridProjection()
        p.reconcile(G, [(2.5,3.5,90)])
        resized = GridGeometry(20, 20, 1., 0., 0.)
        self.assertEqual(p.reconcile(resized, [(2.5,3.5,90)]),
                         ({(2,3):90}, ((2,3),), True))

    def test_outside_map_is_not_painted(self):
        p = AcousticGridProjection()
        self.assertEqual(p.reconcile(G, [(-.1,0.,90),(10.,0.,90),(0.,0.,20)]),
                         ({(0,0):20}, (), True))

    def test_overflow_clears_all_not_partial_map(self):
        p = AcousticGridProjection(max_cells=1)
        p.reconcile(G, [(1.,1.,30)])
        self.assertEqual(p.reconcile(G, [(1.,1.,30),(2.,2.,30)]),
                         ({}, ((1,1),), False))

    def test_lethal_or_unknown_cost_fails_closed(self):
        p = AcousticGridProjection()
        p.reconcile(G, [(1.,1.,40)])
        self.assertEqual(p.reconcile(G, [(1.,1.,253)]),
                         ({}, ((1,1),), False))
        self.assertEqual(p.reconcile(G, [(1.,1.,255)]),
                         ({}, (), False))

    def test_geometry_validation(self):
        with self.assertRaises(ValueError):
            GridGeometry(0, 10, 1., 0., 0.)
        with self.assertRaises(ValueError):
            GridGeometry(10, 10, 0., 0., 0.)

if __name__ == '__main__':
    unittest.main()
