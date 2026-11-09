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
revoke_methods = [n for n in node.body if isinstance(n, ast.FunctionDef)
                  and n.name in ("revoke_audio", "revoke_visual")]
exec(compile(ast.Module(body=[method, *revoke_methods], type_ignores=[]),
             str(SOURCE), "exec"), scope)


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

    def publish_pairs(self):
        # Production publication is independently tested; verify revocation
        # triggers a new publication rather than waiting for a timer.
        self.published.append(("publish", tuple(self.audio), tuple(self.visual)))


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

    def test_malformed_audio_envelope_revokes_only_audio_side(self):
        h = Harness(1_000_000_000, 1_000_000_000)
        visual = h.visual
        scope["revoke_audio"](h)
        self.assertEqual(h.audio, [])
        self.assertIsNone(h.audio_time)
        self.assertIsNone(h.pending_audio)
        self.assertIs(h.visual, visual)
        self.assertEqual(h.temporal.resets, 1)
        self.assertEqual(len(h.published), 1)

    def test_malformed_visual_envelope_revokes_only_visual_side(self):
        h = Harness(1_000_000_000, 1_000_000_000)
        audio = h.audio
        scope["revoke_visual"](h)
        self.assertEqual(h.visual, [])
        self.assertIsNone(h.vision_time)
        self.assertIsNone(h.pending_visual)
        self.assertIs(h.audio, audio)
        self.assertEqual(h.temporal.resets, 1)
        self.assertEqual(len(h.published), 1)

    def test_rejected_envelope_cannot_clear_pending_retry(self):
        # Production callback order: freshness and DDS replay checks must
        # precede any mutation of the pending TF2 snapshot.
        for callback, fence in (("on_audio", "audio_fence"),
                                ("on_visual", "vision_fence")):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            body = handler.body
            pending_mutations = [i for i, statement in enumerate(body)
                                 if isinstance(statement, ast.If)
                                 and isinstance(statement.test, ast.UnaryOp)
                                 and isinstance(statement.test.operand, ast.Name)
                                 and statement.test.operand.id == "retry"
                                 and any(isinstance(n, ast.Attribute)
                                         and n.attr.startswith("pending_")
                                         for n in ast.walk(statement))]
            self.assertEqual(len(pending_mutations), 1, callback)
            mutation_index = pending_mutations[0]
            preceding = body[:mutation_index]
            self.assertTrue(any(isinstance(n, ast.Call)
                                and isinstance(n.func, ast.Attribute)
                                and n.func.attr == "fresh"
                                for stmt in preceding for n in ast.walk(stmt)))
            self.assertTrue(any(isinstance(n, ast.Attribute)
                                and n.attr == fence
                                for stmt in preceding for n in ast.walk(stmt)))

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
