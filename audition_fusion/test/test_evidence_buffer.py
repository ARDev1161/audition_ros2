"""M4 six-ray gate regression tests with known synthetic geometry."""
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from evidence_buffer import DiagnosticEvidenceBuffer
from uncertainty import PlanarRay

SESSION = 'a' * 32


def ray(stamp, x, target=(3., 2.)):
    dx, dy = target[0] - x, target[1]
    magnitude = math.hypot(dx, dy)
    return PlanarRay(stamp, (x, 0.),
                     (dx / magnitude, dy / magnitude),
                     .001, (.01, 0., 0., .01))


def series(second_target=(3., 2.)):
    return ([ray(1_000_000_000 + i * 200_000_000, float(i))
             for i in range(3)]
            + [ray(2_000_000_000 + i * 200_000_000,
                   float(i), second_target) for i in range(3)])


class EvidenceBufferTests(unittest.TestCase):
    def test_stable_source_two_windows_admitted(self):
        buf = DiagnosticEvidenceBuffer()
        for item in series()[:5]:
            self.assertFalse(buf.add(SESSION, 'speaker', 1, item).evaluated)
        result = buf.add(SESSION, 'speaker', 1, series()[5])
        self.assertTrue(result.evaluated)
        self.assertTrue(result.admitted)
        self.assertFalse(buf.tracks)

    def test_moving_emitter_rejected(self):
        buf = DiagnosticEvidenceBuffer()
        results = [buf.add(SESSION, 'speaker', 1, r)
                   for r in series((5., 2.))]
        self.assertTrue(results[-1].evaluated)
        self.assertFalse(results[-1].admitted)

    def test_replayed_sample_and_short_spacing_not_used(self):
        buf = DiagnosticEvidenceBuffer()
        items = series()
        buf.add(SESSION, 'speaker', 1, items[0])
        self.assertFalse(buf.add(SESSION, 'speaker', 1, items[0]).evaluated)
        self.assertFalse(buf.add(SESSION, 'speaker', 1,
                                  ray(items[0].stamp_ns + 1, 0.)).evaluated)
        self.assertEqual(len(next(iter(buf.tracks.values()))), 1)

    def test_session_generation_and_missing_track_retire(self):
        buf = DiagnosticEvidenceBuffer()
        buf.add(SESSION, 'speaker', 1, series()[0])
        buf.add(SESSION, 'speaker', 2, series()[1])
        self.assertEqual(len(buf.tracks), 1)
        self.assertIn((SESSION, 'speaker', 2), buf.tracks)
        buf.retire_missing(set())
        self.assertFalse(buf.tracks)
        buf.add(SESSION, 'speaker', 2, series()[2])
        buf.reset()
        self.assertFalse(buf.tracks)

    def test_bounded_tracks_and_ttl(self):
        buf = DiagnosticEvidenceBuffer(max_sources=1,
                                       maximum_window_ns=1_000_000_000)
        buf.add(SESSION, 'one', 1, series()[0])
        buf.add(SESSION, 'two', 1, series()[0])
        self.assertEqual(len(buf.tracks), 1)
        self.assertIn((SESSION, 'two', 1), buf.tracks)
        buf.add(SESSION, 'two', 1, ray(3_000_000_000, 1.))
        self.assertEqual(len(next(iter(buf.tracks.values()))), 1)

    def test_bad_limits(self):
        with self.assertRaises(ValueError):
            DiagnosticEvidenceBuffer(max_sources=0)


if __name__ == '__main__':
    unittest.main()
