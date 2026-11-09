"""Fail-closed regression tests for production AV publication gates.

Execute the actual AvPairNode.publish_pairs method without ROS imports.
The fake matcher is deliberately permissive: the node must prevent it from
seeing incomplete TF2 snapshots or expired capture-time evidence.
"""
import ast
import pathlib
import time
import types
import unittest

SOURCE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "av_pair_node.py"
tree = ast.parse(SOURCE.read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "AvPairNode")
method = next(n for n in node.body
              if isinstance(n, ast.FunctionDef) and n.name == "publish_pairs")
scope = {"time": time}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), scope)


class Matcher:
    skew_ns = 150_000_000

    def __init__(self):
        self.calls = 0

    def propose(self, audio, visual):
        self.calls += 1
        return ["candidate"] if audio and visual else []


class Logger:
    def warning(self, *_args, **_kwargs):
        pass


class Harness:
    def __init__(self):
        self.age_ns = 800_000_000
        self.future_ns = 100_000_000
        self.audio = [types.SimpleNamespace(stamp_ns=1_900_000_000)]
        self.visual = [types.SimpleNamespace(stamp_ns=1_900_000_000)]
        self.audio_time = time.monotonic()
        self.vision_time = time.monotonic()
        self.pending_audio = None
        self.pending_visual = None
        self.matcher = Matcher()
        self.emitted = []

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=2_000_000_000))

    def get_logger(self):
        return Logger()

    def emit(self, pairs, pending=False):
        self.emitted.append((pairs, pending))


class TestPublicationGate(unittest.TestCase):
    def publish(self, h):
        scope["publish_pairs"](h)
        self.assertEqual(len(h.emitted), 1)
        return h.emitted[0]

    def test_fresh_complete_pair_reaches_matcher(self):
        h = Harness()
        self.assertEqual(self.publish(h), (["candidate"], False))
        self.assertEqual(h.matcher.calls, 1)

    def test_pending_audio_tf_blocks_all_association(self):
        h = Harness()
        h.pending_audio = object()
        self.assertEqual(self.publish(h), ([], True))
        self.assertEqual(h.matcher.calls, 0)

    def test_pending_visual_tf_blocks_all_association(self):
        h = Harness()
        h.pending_visual = object()
        self.assertEqual(self.publish(h), ([], True))
        self.assertEqual(h.matcher.calls, 0)

    def test_capture_timestamp_expiry_blocks_matcher_even_if_recently_received(self):
        h = Harness()
        h.audio[0].stamp_ns = 1_000_000_000
        self.assertEqual(self.publish(h), ([], False))
        self.assertEqual(h.matcher.calls, 0)

    def test_future_capture_stamp_blocks_matcher(self):
        h = Harness()
        h.visual[0].stamp_ns = 2_200_000_000
        self.assertEqual(self.publish(h), ([], False))
        self.assertEqual(h.matcher.calls, 0)

    def test_monotonic_arrival_expiry_blocks_matcher(self):
        h = Harness()
        h.vision_time = time.monotonic() - 2.0
        self.assertEqual(self.publish(h), ([], False))
        self.assertEqual(h.matcher.calls, 0)

    def test_skewed_fresh_snapshots_preserve_pending_evidence(self):
        h = Harness()
        h.visual[0].stamp_ns = 1_500_000_000
        self.assertEqual(self.publish(h), ([], True))
        self.assertEqual(h.matcher.calls, 1)

    def test_missing_counterpart_never_produces_candidate(self):
        h = Harness()
        h.visual = []
        self.assertEqual(self.publish(h), ([], False))
        self.assertEqual(h.matcher.calls, 1)


if __name__ == "__main__":
    unittest.main()
