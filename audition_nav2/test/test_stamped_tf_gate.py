"""M5 stamped transform contract tests."""
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nav_admission import MetricSource
from stamped_tf_gate import StampedTransformGate
from soft_cost_cells import SoftCostCells

T = 2_000_000_000
COV = (.04, 0., 0., 0., .04, 0., 0., 0., .04)

def source(validated=True):
    return MetricSource('s1', 'microphone_array', T-10, T+100,
                        (1., 2., 0.), COV, validated)

class StampedTfGateTests(unittest.TestCase):
    def test_lookup_receives_observation_stamp_not_now(self):
        calls = []
        def lookup(target, frame, stamp, xyz):
            calls.append((target, frame, stamp, xyz))
            return (3., 4.)
        gate = StampedTransformGate('map', lookup)
        self.assertEqual(gate(source(), T), (3.,4.))
        self.assertEqual(calls, [('map','microphone_array',T-10,(1.,2.,0.))])

    def test_invalid_or_unvalidated_never_reaches_lookup(self):
        calls = []
        gate = StampedTransformGate('map', lambda *args: calls.append(args))
        with self.assertRaises(ValueError):
            gate(source(False), T)
        self.assertEqual(calls, [])

    def test_missing_tf_clears_previous_cost(self):
        gate = StampedTransformGate('map', lambda *args: (3.,4.))
        cells = SoftCostCells(resolution_m=1)
        self.assertEqual(cells.reconcile([source()], T, transform=lambda s:gate(s,T))[0], {(3,4):100})
        missing = StampedTransformGate('map', lambda *args: (_ for _ in ()).throw(LookupError('TF unavailable')))
        self.assertEqual(cells.reconcile([source()], T, transform=lambda s:missing(s,T)), ({}, ((3,4),)))

    def test_reject_nan_and_out_of_bounds_transform(self):
        for xy in [(float('nan'), 1.), (1001., 0.), (1.,)]:
            gate = StampedTransformGate('map', lambda *args: xy)
            with self.assertRaises(ValueError):
                gate(source(), T)

    def test_reject_invalid_target_frame(self):
        with self.assertRaises(ValueError):
            StampedTransformGate('/map', lambda *args: (1.,2.))

if __name__ == '__main__':
    unittest.main()
