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

    def test_rejected_envelopes_cannot_revoke_newer_sensor_state(self):
        # A stale, malformed or replayed envelope must not erase a newer
        # accepted observation. Check the actual production callback AST.
        for callback, revoker in (("on_audio", "revoke_audio"),
                                  ("on_visual", "revoke_visual")):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            revocations = [n for n in ast.walk(handler)
                           if isinstance(n, ast.Call)
                           and isinstance(n.func, ast.Attribute)
                           and n.func.attr == revoker]
            self.assertFalse(revocations, callback)
            # The first callback guard must reject malformed/expired
            # envelopes without mutating evidence or the pending TF retry.
            first_guard = next(n for n in handler.body if isinstance(n, ast.If))
            self.assertTrue(any(isinstance(n, ast.Call)
                                and isinstance(n.func, ast.Attribute)
                                and n.func.attr == "fresh"
                                for n in ast.walk(first_guard.test)))
            self.assertTrue(any(isinstance(n, ast.Return)
                                for n in first_guard.body))
            self.assertFalse(any(isinstance(n, ast.Assign)
                                 for n in first_guard.body))

    def test_production_callbacks_ignore_stale_without_state_mutation(self):
        # Execute the production early-rejection path, rather than only
        # inspecting its AST. A delayed packet must not clear a newer
        # accepted snapshot or a pending TF2 transform.
        for callback in ("on_audio", "on_visual"):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            namespace = {"ns": lambda stamp: stamp.sec * 1000000000 + stamp.nanosec}
            exec(compile(ast.Module(body=[handler], type_ignores=[]),
                         str(SOURCE), "exec"), namespace)
            h = Harness(1_000_000_000, 1_000_000_000)
            h.clock = lambda: h.current
            h.fresh = lambda stamp, now_ns: False
            h.get_logger = lambda: types.SimpleNamespace(warning=lambda *args, **kwargs: None)
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0),
                    frame_id="sensor"))
            before = (h.audio, h.visual, h.pending_audio, h.pending_visual,
                      h.audio_time, h.vision_time)
            namespace[callback](h, msg)
            after = (h.audio, h.visual, h.pending_audio, h.pending_visual,
                     h.audio_time, h.vision_time)
            self.assertEqual(before, after, callback)
            self.assertEqual(h.published, [], callback)
            self.assertEqual(h.temporal.resets, 0, callback)

    def test_production_callbacks_ignore_replayed_sequence_without_mutation(self):
        # Execute the production DDS rejection path with a fresh envelope.
        # In particular, a replay must not replace a pending TF2 snapshot.
        for callback, fence_name, session_name in (
                ("on_audio", "audio_fence", "scene_session_id"),
                ("on_visual", "vision_fence", "vision_session_id")):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            namespace = {"ns": lambda stamp: stamp.sec * 1000000000 + stamp.nanosec}
            exec(compile(ast.Module(body=[handler], type_ignores=[]),
                         str(SOURCE), "exec"), namespace)
            h = Harness(1_000_000_000, 1_000_000_000)
            h.clock = lambda: h.current
            h.fresh = lambda stamp, now_ns: True
            h.get_logger = lambda: types.SimpleNamespace(
                warning=lambda *args, **kwargs: None)
            fence = types.SimpleNamespace(accept=lambda *args: False,
                                          session="session", sequence=7)
            setattr(h, fence_name, fence)
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0),
                    frame_id="sensor"),
                sequence_number=7)
            setattr(msg, session_name, "session")
            before = (h.audio, h.visual, h.pending_audio, h.pending_visual,
                      h.audio_time, h.vision_time)
            namespace[callback](h, msg)
            after = (h.audio, h.visual, h.pending_audio, h.pending_visual,
                     h.audio_time, h.vision_time)
            self.assertEqual(before, after, callback)
            self.assertEqual(h.published, [], callback)
            self.assertEqual(h.temporal.resets, 0, callback)

    def test_overcapacity_revokes_temporal_confirmations(self):
        # Exercise the production fast-path for both modalities. The old
        # observations must be cleared AND their confirmations retired.
        for callback, field, collection in (
                ("on_audio", "audio", "sources"),
                ("on_visual", "visual", "tracks")):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            namespace = {"ns": lambda stamp: stamp.sec * 1000000000 + stamp.nanosec,
                         "time": types.SimpleNamespace(monotonic=lambda: 42.0)}
            exec(compile(ast.Module(body=[handler], type_ignores=[]),
                         str(SOURCE), "exec"), namespace)
            h = Harness(1_000_000_000, 1_000_000_000)
            h.clock = lambda: h.current
            h.fresh = lambda stamp, now_ns: True
            h.matcher = types.SimpleNamespace(maximum_tracks=2)
            h.get_logger = lambda: types.SimpleNamespace(
                warning=lambda *args, **kwargs: None)
            fence = types.SimpleNamespace(accept=lambda *args: True)
            setattr(h, "audio_fence" if callback == "on_audio" else "vision_fence", fence)
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0),
                    frame_id="sensor"),
                sequence_number=8)
            setattr(msg, "scene_session_id" if callback == "on_audio"
                    else "vision_session_id", "session")
            setattr(msg, collection, [object(), object(), object()])
            namespace[callback](h, msg)
            self.assertEqual(getattr(h, field), [], callback)
            self.assertEqual(h.temporal.resets, 1, callback)
            self.assertEqual(len(h.published), 1, callback)
            self.assertIsNone(getattr(h, "pending_" + field), callback)

    def test_overcapacity_does_not_preserve_old_confirmation(self):
        # A rejected snapshot must not leave temporal confirmations alive,
        # even if the opposite modality remains available.
        for callback, collection in (("on_audio", "sources"),
                                     ("on_visual", "tracks")):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            namespace = {
                "ns": lambda stamp: stamp.sec * 1000000000 + stamp.nanosec,
                "time": types.SimpleNamespace(monotonic=lambda: 42.0)}
            exec(compile(ast.Module(body=[handler], type_ignores=[]),
                         str(SOURCE), "exec"), namespace)
            h = Harness(1_000_000_000, 1_000_000_000)
            h.clock = lambda: h.current
            h.fresh = lambda stamp, now_ns: True
            h.matcher = types.SimpleNamespace(maximum_tracks=1)
            h.get_logger = lambda: types.SimpleNamespace(
                warning=lambda *args, **kwargs: None)
            setattr(h, "audio_fence" if callback == "on_audio"
                    else "vision_fence",
                    types.SimpleNamespace(accept=lambda *args: True))
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0),
                    frame_id="sensor"),
                sequence_number=1)
            setattr(msg, "scene_session_id" if callback == "on_audio"
                    else "vision_session_id", "session")
            setattr(msg, collection, [object(), object()])
            opposite = h.visual if callback == "on_audio" else h.audio
            namespace[callback](h, msg)
            self.assertEqual(h.temporal.resets, 1)
            self.assertEqual(len(h.published), 1)
            self.assertIs(h.visual if callback == "on_audio" else h.audio,
                          opposite)

    def test_overcapacity_rejects_without_transforming_untrusted_tracks(self):
        # The bounded fast-path must run before per-track parsing or TF2.
        # Malformed excess entries cannot trigger arbitrary transformations.
        for callback, collection in (("on_audio", "sources"),
                                     ("on_visual", "tracks")):
            handler = next(n for n in node.body
                           if isinstance(n, ast.FunctionDef) and n.name == callback)
            namespace = {
                "ns": lambda stamp: stamp.sec * 1000000000 + stamp.nanosec,
                "time": types.SimpleNamespace(monotonic=lambda: 42.0)}
            exec(compile(ast.Module(body=[handler], type_ignores=[]),
                         str(SOURCE), "exec"), namespace)
            h = Harness(1_000_000_000, 1_000_000_000)
            h.clock = lambda: h.current
            h.fresh = lambda stamp, now_ns: True
            h.matcher = types.SimpleNamespace(maximum_tracks=1)
            h.get_logger = lambda: types.SimpleNamespace(
                warning=lambda *args, **kwargs: None)
            h.transform_ray = lambda *args: self.fail("overcapacity reached TF2")
            setattr(h, "audio_fence" if callback == "on_audio"
                    else "vision_fence",
                    types.SimpleNamespace(accept=lambda *args: True))
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=1, nanosec=0),
                    frame_id="sensor"),
                sequence_number=1)
            setattr(msg, "scene_session_id" if callback == "on_audio"
                    else "vision_session_id", "session")
            setattr(msg, collection, [object(), object()])
            namespace[callback](h, msg)
            self.assertEqual(h.temporal.resets, 1)
            self.assertEqual(len(h.published), 1)

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
