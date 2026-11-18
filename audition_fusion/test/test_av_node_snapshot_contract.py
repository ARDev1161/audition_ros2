"""Regression: malformed active competitors must invalidate whole AV snapshots.

Extract node callbacks from source to exercise admission without a ROS installation.
The production callbacks (not reimplementations) run with stub transport objects.
"""
import ast
import pathlib
import time
import types
import unittest

SOURCE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "av_pair_node.py"
tree = ast.parse(SOURCE.read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "AvPairNode")
callbacks = [n for n in node.body if isinstance(n, ast.FunctionDef)
             and n.name in ("on_audio", "on_visual")]
scope = {
    "time": time,
    "AcousticSource": types.SimpleNamespace(ACTIVE=1),
    "ns": lambda stamp: stamp,
    "Ray": lambda *args: args,
}
exec(compile(ast.Module(body=callbacks, type_ignores=[]), str(SOURCE), "exec"), scope)


class Fence:
    session = "a" * 32
    sequence = 1

    def accept(self, *args):
        return True


class Logger:
    def warning(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass


class Harness:
    def __init__(self):
        self.audio_fence = Fence()
        self.vision_fence = Fence()
        self.matcher = types.SimpleNamespace(maximum_tracks=16)
        self.temporal = types.SimpleNamespace(reset=self.reset)
        self.audio = []
        self.visual = []
        self.pending_audio = None
        self.pending_visual = None
        self.resets = 0
        self.published = 0

    def reset(self):
        self.resets += 1

    def clock(self):
        return 1000

    def fresh(self, stamp, now):
        return True

    def observe(self, envelope, sample, now):
        return True

    def transform_ray(self, frame, stamp, vector):
        return ((0., 0., 0.), (1., 0., 0.))

    def get_logger(self):
        return Logger()

    def publish_pairs(self):
        self.published += 1


def audio(source_id, valid=True):
    return types.SimpleNamespace(
        source_id=source_id, state=1, has_direction=valid,
        has_angular_variance=True, source_generation=1,
        header=types.SimpleNamespace(frame_id=""),
        last_seen=900, direction_unit=(1., 0., 0.),
        angular_variance_rad2=0.01)


def visual(track_id, valid=True):
    return types.SimpleNamespace(
        visual_track_id=track_id, active=True,
        activity_evidence=valid, has_angular_variance=True,
        visual_track_generation=1, observed_at=900,
        direction_unit=(1., 0., 0.), angular_variance_rad2=0.01)


class TestMalformedCompetitor(unittest.TestCase):
    def test_audio_invalid_competitor_clears_entire_snapshot(self):
        h = Harness()
        msg = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=1000, frame_id="mic"),
            scene_session_id="a" * 32, sequence_number=1,
            sources=[audio("good"), audio("bad", False)])
        scope["on_audio"](h, msg)
        self.assertEqual(h.audio, [])
        self.assertEqual(h.resets, 1)
        self.assertEqual(h.published, 1)
        self.assertIsNone(h.pending_audio)

    def test_visual_invalid_competitor_clears_entire_snapshot(self):
        h = Harness()
        msg = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=1000, frame_id="camera"),
            vision_session_id="b" * 32, sequence_number=1,
            tracks=[visual("good"), visual("bad", False)])
        scope["on_visual"](h, msg)
        self.assertEqual(h.visual, [])
        self.assertEqual(h.resets, 1)
        self.assertEqual(h.published, 1)
        self.assertIsNone(h.pending_visual)

    def test_valid_snapshots_still_admitted(self):
        h = Harness()
        a = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=1000, frame_id="mic"),
            scene_session_id="a" * 32, sequence_number=1,
            sources=[audio("good")])
        scope["on_audio"](h, a)
        self.assertEqual(len(h.audio), 1)
        v = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=1000, frame_id="camera"),
            vision_session_id="b" * 32, sequence_number=1,
            tracks=[visual("good")])
        scope["on_visual"](h, v)
        self.assertEqual(len(h.visual), 1)
        self.assertEqual(h.resets, 0)


if __name__ == "__main__":
    unittest.main()
