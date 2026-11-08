"""Regression for TF2 timeout retirement and asymmetric AV snapshot expiry.

Exercise the production timer method without importing ROS 2. A timed-out
transform must revoke temporal evidence but must not discard the other
sensor's still-fresh snapshot.
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
              if isinstance(n, ast.FunctionDef) and n.name == "on_timer")
scope = {"time": time}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), scope)


class Harness:
    def __init__(self):
        self.age_ns = 800_000_000
        self.future_ns = 100_000_000
        self.audio = []
        self.visual = []
        self.audio_time = None
        self.vision_time = None
        self.pending_audio = None
        self.pending_visual = None
        self.had_pairs = False
        self.reset_count = 0
        self.emitted = []
        self.retries = []
        self.temporal = types.SimpleNamespace(reset=self.reset)

    def reset(self):
        self.reset_count += 1

    def clock(self):
        return 2_000_000_000

    def fresh(self, stamp, now):
        return stamp > 0 and now - stamp <= self.age_ns

    def emit(self, pairs):
        self.emitted.append(pairs)
        self.had_pairs = bool(pairs)

    def on_audio(self, msg, retry=False):
        assert retry, "timer must not re-admit the stream fence"
        self.retries.append(("audio", msg))

    def on_visual(self, msg, retry=False):
        assert retry, "timer must not re-admit the stream fence"
        self.retries.append(("visual", msg))


def ray(stamp):
    return types.SimpleNamespace(stamp_ns=stamp)


def pending(stamp):
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp))


class TestTimerRetirement(unittest.TestCase):
    def test_expired_audio_tf_preserves_fresh_visual(self):
        h = Harness()
        h.pending_audio = pending(1_000_000_000)
        h.audio = [ray(1_000_000_000)]
        h.audio_time = time.monotonic()
        h.visual = [ray(1_900_000_000)]
        h.vision_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertIsNone(h.pending_audio)
        self.assertEqual(h.audio, [])
        self.assertIsNone(h.audio_time)
        self.assertEqual(len(h.visual), 1)
        self.assertIsNotNone(h.vision_time)
        self.assertGreaterEqual(h.reset_count, 1)
        self.assertEqual(h.emitted, [[]])

    def test_expired_visual_tf_preserves_fresh_audio(self):
        h = Harness()
        h.pending_visual = pending(1_000_000_000)
        h.visual = [ray(1_000_000_000)]
        h.vision_time = time.monotonic()
        h.audio = [ray(1_900_000_000)]
        h.audio_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertIsNone(h.pending_visual)
        self.assertEqual(h.visual, [])
        self.assertIsNone(h.vision_time)
        self.assertEqual(len(h.audio), 1)
        self.assertIsNotNone(h.audio_time)
        self.assertGreaterEqual(h.reset_count, 1)
        self.assertEqual(h.emitted, [[]])

    def test_capture_age_expires_only_old_side(self):
        h = Harness()
        h.audio = [ray(1_000_000_000)]
        h.visual = [ray(1_900_000_000)]
        h.audio_time = time.monotonic()
        h.vision_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.audio, [])
        self.assertEqual(len(h.visual), 1)
        self.assertEqual(h.emitted, [[]])
        self.assertEqual(h.reset_count, 1)

    def test_fresh_snapshots_are_not_retired(self):
        h = Harness()
        h.audio = [ray(1_900_000_000)]
        h.visual = [ray(1_900_000_000)]
        h.audio_time = time.monotonic()
        h.vision_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(len(h.audio), 1)
        self.assertEqual(len(h.visual), 1)
        self.assertEqual(h.reset_count, 0)
        self.assertEqual(h.emitted, [])


    def test_fresh_pending_tf_is_retried_without_refreshing_stamp(self):
        h = Harness()
        original = pending(1_900_000_000)
        h.pending_audio = original
        h.visual = [ray(1_900_000_000)]
        h.vision_time = time.monotonic()
        scope["on_timer"](h)
        self.assertEqual(h.retries, [("audio", original)])
        self.assertIsNone(h.pending_audio)
        self.assertEqual(original.header.stamp, 1_900_000_000)
        self.assertEqual(len(h.visual), 1)
        self.assertEqual(h.emitted, [])

    def test_expired_pending_audio_does_not_block_fresh_visual_retry(self):
        h = Harness()
        h.pending_audio = pending(1_000_000_000)
        visual = pending(1_900_000_000)
        h.pending_visual = visual
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.retries, [("visual", visual)])
        self.assertIsNone(h.pending_audio)
        self.assertIsNone(h.pending_visual)
        self.assertEqual(h.emitted, [[]])
        self.assertGreaterEqual(h.reset_count, 1)

    def test_both_expired_pending_never_retry_or_reconfirm(self):
        h = Harness()
        h.pending_audio = pending(1_000_000_000)
        h.pending_visual = pending(1_000_000_000)
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.retries, [])
        self.assertIsNone(h.pending_audio)
        self.assertIsNone(h.pending_visual)
        self.assertEqual(h.emitted, [[]])
        self.assertGreaterEqual(h.reset_count, 1)

    def test_fresh_audio_and_visual_pending_both_retry(self):
        h = Harness()
        audio = pending(1_900_000_000)
        visual = pending(1_900_000_000)
        h.pending_audio = audio
        h.pending_visual = visual
        scope["on_timer"](h)
        self.assertEqual(h.retries, [("audio", audio), ("visual", visual)])
        self.assertEqual(h.emitted, [])


    def test_future_dated_audio_ray_is_retired_without_losing_visual(self):
        h = Harness()
        h.audio = [ray(2_200_000_000)]  # beyond permitted future skew
        h.visual = [ray(1_900_000_000)]
        h.audio_time = time.monotonic()
        h.vision_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.audio, [])
        self.assertIsNone(h.audio_time)
        self.assertEqual(len(h.visual), 1)
        self.assertEqual(h.emitted, [[]])
        self.assertGreaterEqual(h.reset_count, 1)

    def test_future_dated_visual_ray_is_retired_without_losing_audio(self):
        h = Harness()
        h.visual = [ray(2_200_000_000)]
        h.audio = [ray(1_900_000_000)]
        h.vision_time = time.monotonic()
        h.audio_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.visual, [])
        self.assertIsNone(h.vision_time)
        self.assertEqual(len(h.audio), 1)
        self.assertEqual(h.emitted, [[]])

    def test_monotonic_timeout_retires_only_old_audio_snapshot(self):
        h = Harness()
        h.audio = [ray(1_900_000_000)]
        h.visual = [ray(1_900_000_000)]
        h.audio_time = time.monotonic() - 2.0
        h.vision_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.audio, [])
        self.assertIsNone(h.audio_time)
        self.assertEqual(len(h.visual), 1)
        self.assertEqual(h.emitted, [[]])

    def test_revocation_is_not_republished_on_next_timer_tick(self):
        h = Harness()
        h.audio = [ray(1_000_000_000)]
        h.visual = [ray(1_900_000_000)]
        h.audio_time = time.monotonic()
        h.vision_time = time.monotonic()
        h.had_pairs = True
        scope["on_timer"](h)
        self.assertEqual(h.emitted, [[]])
        scope["on_timer"](h)
        self.assertEqual(h.emitted, [[]])


if __name__ == "__main__":
    unittest.main()
