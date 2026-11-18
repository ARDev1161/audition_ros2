import importlib.util
import math
import pathlib
import unittest

path = pathlib.Path(__file__).resolve().parents[1] / 'geometry.py'
spec = importlib.util.spec_from_file_location('fusion_geometry', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GeometryTests(unittest.TestCase):
    def test_translation_required(self):
        self.assertIsNone(module.triangulate_2d((0, 0), (1, 0), (0, 0), (0, 1)))

    def test_intersection(self):
        result = module.triangulate_2d((0, 0), (1, 0), (1, -1), (0, 1))
        self.assertAlmostEqual(result[0], 1)
        self.assertAlmostEqual(result[1], 0)

    def test_parallel_and_backward(self):
        self.assertIsNone(module.triangulate_2d((0, 0), (1, 0), (1, 1), (1, 0)))
        self.assertIsNone(module.triangulate_2d((0, 0), (-1, 0), (1, -1), (0, 1)))

    def test_invalid_and_ill_conditioned(self):
        self.assertIsNone(module.triangulate_2d((0, 0), (float('nan'), 0), (1, 1), (0, -1)))
        self.assertIsNone(module.triangulate_2d((0, 0), (1, 0), (1, -1), (0.999, 0.0447)))
        self.assertIsNone(module.triangulate_2d((0, 0), (2, 0), (1, -1), (0, 1)))


if __name__ == '__main__':
    unittest.main()
