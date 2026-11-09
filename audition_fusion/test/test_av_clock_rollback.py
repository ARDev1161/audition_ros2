"""Regression of production ROS clock rollback isolation.

A /clock rewind must invalidate both sensor fences, pending TF retries,
temporal confirmations and previously published pair hypotheses. The
test invokes the production clock() method, not a reimplementation.
"""
import ast
import pathlib
import types
import unittest

SOURCE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "av_pair_node.py"
tree = ast.parse(SOURCE.read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "AvPairNode")
method = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == "clock")
scope = {}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), scope)


class Fence:
    def __init__(self):
        self.resets = 0

    def reset(self):
        self.resets += 1


class Temporal:
    def __init__(self):
        self.resets = 0

    def reset(self):
        self.resets += 1


class Harness:
    def __init__(self, previous, current):
        self.last_clock_ns = previous
        self.current = current
        self.audio_fence = Fence()
        self.vision_fence = Fence()
        self.audio = [object()]
        self.visual = [object()]
        self.pending_audio = object()
        self.pending_visual = object()
        self.audio_time = 1.0
        self.vision_time = 1.0
        self.temporal = Temporal()
        self.published = []

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=self.current))

    def emit(self, pairs):
        self.published.append(pairs)


class TestClockRollback(unittest.TestCase):
    def test_rewind_revokes_all_sensor_evidence(self):
        h = Harness(2_000_000_000, 1_000_000_000)
        self.assertEqual(scope["clock"](h), 1_000_000_000)
        self.assertEqual((h.audio_fence.resets, h.vision_fence.resets), (1, 1))
        self.assertEqual(h.temporal.resets, 1)
        self.assertEqual((h.audio, h.visual), ([], []))
        self.assertIsNone(h.pending_audio)
        self.assertIsNone(h.pending_visual)
        self.assertIsNone(h.audio_time)
        self.assertIsNone(h.vision_time)
        self.assertEqual(h.published, [[]])
        self.assertEqual(h.last_clock_ns, 1_000_000_000)

    def test_forward_time_preserves_all_evidence(self):
        h = Harness(1_000_000_000, 2_000_000_000)
        audio, visual = h.audio, h.visual
        pending_audio, pending_visual = h.pending_audio, h.pending_visual
        self.assertEqual(scope["clock"](h), 2_000_000_000)
        self.assertIs(h.audio, audio)
        self.assertIs(h.visual, visual)
        self.assertIs(h.pending_audio, pending_audio)
        self.assertIs(h.pending_visual, pending_visual)
        self.assertEqual((h.audio_fence.resets, h.vision_fence.resets,
                          h.temporal.resets), (0, 0, 0))
        self.assertEqual(h.published, [])

    def test_equal_time_does_not_rewind(self):
        h = Harness(2_000_000_000, 2_000_000_000)
        scope["clock"](h)
        self.assertEqual(h.published, [])
        self.assertEqual(h.temporal.resets, 0)

    def test_first_clock_sample_does_not_revoke(self):
        h = Harness(None, 2_000_000_000)
        scope["clock"](h)
        self.assertEqual(h.published, [])
        self.assertEqual(h.last_clock_ns, 2_000_000_000)


if __name__ == "__main__":
    unittest.main()
