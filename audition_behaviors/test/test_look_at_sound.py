#!/usr/bin/env python3
"""Pure policy regression for LookAtSound freshness and source matching."""
import importlib.util
import pathlib
import math
import unittest
from unittest.mock import patch

from audition_msgs.msg import AttentionTarget

path = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "look_at_sound.py"
spec = importlib.util.spec_from_file_location("look_at_sound", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FreshnessTest(unittest.TestCase):
    def setUp(self):
        self.target = AttentionTarget()
        self.target.valid = True
        self.target.source_id = "voice-a"
        self.target.header.frame_id = "base_link"
        self.target.direction_unit.x = 1.0

    def check(self, now=1.1, arrival=1.0, source="", frame="base_link"):
        return module.fresh_target(self.target, arrival, now, 0.5, source, frame)

    def test_confirmation_evidence(self):
        first = self.target
        second = AttentionTarget()
        second.valid = True
        second.source_id = "voice-a"
        second.scene_generation = first.scene_generation
        second.header.frame_id = "base_link"
        second.direction_unit.x = 1.0
        count, anchor = module.accumulate_bearing_evidence(0, None, first, 0.2)
        self.assertEqual(count, 1)
        count, anchor = module.accumulate_bearing_evidence(count, anchor, second, 0.2)
        self.assertEqual(count, 2)
        self.assertIs(anchor, first)
        second.source_id = "voice-b"
        count, anchor = module.accumulate_bearing_evidence(count, anchor, second, 0.2)
        self.assertEqual(count, 1)
        self.assertIs(anchor, second)

    def test_confirmation_bearing_anchor_and_generation(self):
        first = self.target
        second = AttentionTarget()
        second.source_id = first.source_id
        second.header.frame_id = "base_link"
        second.direction_unit.x = math.cos(0.3)
        second.direction_unit.y = math.sin(0.3)
        count, anchor = module.accumulate_bearing_evidence(1, first, second, 0.2)
        self.assertEqual(count, 1)
        self.assertIs(anchor, second)
        second.scene_generation += 1
        count, anchor = module.accumulate_bearing_evidence(2, first, second, 0.0)
        self.assertEqual(count, 1)

    def test_confirmation_restarts_on_source_generation_change(self):
        first = self.target
        first.source_id = "tracked"
        first.source_generation = 4
        first.scene_generation = 8
        second = AttentionTarget()
        second.source_id = "tracked"
        second.source_generation = 5
        second.scene_generation = 8
        second.header.frame_id = "base_link"
        second.direction_unit.x = 1.0
        count, anchor = module.accumulate_bearing_evidence(2, first, second, 0.0)
        self.assertEqual(count, 1)
        self.assertIs(anchor, second)

    def test_fresh(self):
        self.assertTrue(self.check())
        self.assertTrue(self.check(source="voice-a"))

    def test_goal_requires_new_observation(self):
        self.assertFalse(module.fresh_target(self.target, 1.0, 1.1, 0.5, "", "base_link", 5, 5))
        self.assertFalse(module.fresh_target(self.target, 1.0, 1.1, 0.5, "", "base_link", 4, 5))
        self.assertTrue(module.fresh_target(self.target, 1.0, 1.1, 0.5, "", "base_link", 6, 5))

    def test_stamp_replay(self):
        self.assertFalse(module.newer_stamp(self.target, None))
        self.target.header.stamp.sec = 5
        self.assertTrue(module.newer_stamp(self.target, None))
        self.assertFalse(module.newer_stamp(self.target, (0, 5, 0)))
        self.target.header.stamp.sec = 6
        self.assertTrue(module.newer_stamp(self.target, (0, 5, 0)))

    def test_watermark_admission(self):
        self.target.header.stamp.sec = 12
        self.assertTrue(module.admissible_observation(self.target, "base_link"))
        self.target.header.frame_id = "map"
        self.assertFalse(module.admissible_observation(self.target, "base_link"))
        self.target.header.frame_id = "base_link"
        self.target.direction_unit.x = float("inf")
        self.assertFalse(module.admissible_observation(self.target, "base_link"))
        self.target.direction_unit.x = 1.0
        self.target.valid = False
        self.assertFalse(module.admissible_observation(self.target, "base_link"))

    def test_ingress_replay_cannot_replace_newer_target(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            latest = None
            last_stamp = None
            sequence = 0
            arrival = 0.0

        server = Server()
        self.target.header.stamp.sec = 20
        module.LookAtSoundServer.on_target(server, self.target)
        self.assertEqual(server.sequence, 1)
        self.assertEqual(server.last_stamp, (0, 20, 0))
        older = AttentionTarget()
        older.valid = True
        older.header.frame_id = "base_link"
        older.direction_unit.x = 1.0
        older.header.stamp.sec = 19
        module.LookAtSoundServer.on_target(server, older)
        self.assertIs(server.latest, self.target)
        self.assertEqual(server.sequence, 1)
        same = AttentionTarget()
        same.valid = True
        same.header.frame_id = "base_link"
        same.direction_unit.x = 1.0
        same.header.stamp.sec = 20
        module.LookAtSoundServer.on_target(server, same)
        self.assertIs(server.latest, self.target)
        self.assertEqual(server.sequence, 1)
        invalid = AttentionTarget()
        invalid.valid = False
        invalid.header.frame_id = "base_link"
        invalid.header.stamp.sec = 20
        invalid.header.stamp.nanosec = 1
        module.LookAtSoundServer.on_target(server, invalid)
        self.assertIsNone(server.latest)
        self.assertEqual(server.last_stamp, (0, 20, 0))
        self.assertEqual(server.sequence, 1)
        newer = AttentionTarget()
        newer.valid = True
        newer.header.frame_id = "base_link"
        newer.direction_unit.x = 1.0
        newer.header.stamp.sec = 21
        module.LookAtSoundServer.on_target(server, newer)
        self.assertIs(server.latest, newer)
        self.assertEqual(server.sequence, 2)
        stale_invalid = AttentionTarget()
        stale_invalid.valid = False
        stale_invalid.header.frame_id = "base_link"
        stale_invalid.header.stamp.sec = 19
        module.LookAtSoundServer.on_target(server, stale_invalid)
        self.assertIs(server.latest, newer)
        current_invalid = AttentionTarget()
        current_invalid.valid = False
        current_invalid.header.frame_id = "base_link"
        current_invalid.header.stamp.sec = 22
        module.LookAtSoundServer.on_target(server, current_invalid)
        self.assertIsNone(server.latest)
        self.assertEqual(server.last_stamp, (0, 21, 0))
        recovery = AttentionTarget()
        recovery.valid = True
        recovery.header.frame_id = "base_link"
        recovery.direction_unit.x = 1.0
        recovery.header.stamp.sec = 23
        module.LookAtSoundServer.on_target(server, recovery)
        self.assertIs(server.latest, recovery)
        self.assertEqual(server.sequence, 3)

    def test_revocation_watermark_rejects_delayed_valid_replay(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            latest = None
            last_stamp = None
            revoked_stamp = None
            sequence = 0
            arrival = 0.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0

        server = Server()
        def observation(stamp, valid=True):
            msg = AttentionTarget()
            msg.valid = valid
            msg.source_id = "voice-a"
            msg.header.frame_id = "base_link"
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = 1.0
            return msg

        module.LookAtSoundServer.on_target(server, observation(100))
        module.LookAtSoundServer.on_target(server, observation(105, False))
        self.assertEqual(server.revoked_stamp, (0, 105, 0))
        self.assertIsNone(server.latest)
        module.LookAtSoundServer.on_target(server, observation(103))
        module.LookAtSoundServer.on_target(server, observation(105))
        self.assertIsNone(server.latest, "delayed valid data resurrected revoked bearing")
        self.assertEqual(server.sequence, 1)
        module.LookAtSoundServer.on_target(server, observation(104, False))
        self.assertEqual(server.revoked_stamp, (0, 105, 0))
        module.LookAtSoundServer.on_target(server, observation(106))
        self.assertEqual(server.sequence, 2)
        self.assertEqual(server.latest.header.stamp.sec, 106)
        module.LookAtSoundServer.on_target(server, observation(105, False))
        self.assertEqual(server.latest.header.stamp.sec, 106)

    def test_foreign_frame_high_generation_cannot_poison_revocation(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            latest = None
            last_stamp = None
            revoked_stamp = None
            sequence = 0
            arrival = 0.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0

        server = Server()
        valid = AttentionTarget()
        valid.valid = True
        valid.header.frame_id = "base_link"
        valid.header.stamp.sec = 10
        valid.direction_unit.x = 1.0
        module.LookAtSoundServer.on_target(server, valid)
        poisoned = AttentionTarget()
        poisoned.valid = False
        poisoned.header.frame_id = "map"
        poisoned.scene_generation = 999999
        poisoned.header.stamp.sec = 999999
        module.LookAtSoundServer.on_target(server, poisoned)
        self.assertIs(server.latest, valid)
        self.assertIsNone(server.revoked_stamp)
        recovered = AttentionTarget()
        recovered.valid = True
        recovered.header.frame_id = "base_link"
        recovered.header.stamp.sec = 11
        recovered.direction_unit.x = 1.0
        module.LookAtSoundServer.on_target(server, recovered)
        self.assertIs(server.latest, recovered)
        self.assertEqual(server.sequence, 2)

    def test_rejected_valid_bearings_do_not_revoke_or_poison_watermarks(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.5
            max_elevation = 0.5
            latest = None
            last_stamp = None
            revoked_stamp = None
            sequence = 0
            arrival = 0.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0

        server = Server()

        def bearing(stamp, x=1.0, y=0.0, valid=True):
            msg = AttentionTarget()
            msg.valid = valid
            msg.header.frame_id = "base_link"
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = x
            msg.direction_unit.y = y
            return msg

        first = bearing(10)
        module.LookAtSoundServer.on_target(server, first)
        server.goal_evidence = first
        server.goal_evidence_count = 2
        server.goal_evidence_sequence = server.sequence

        # A syntactically valid out-of-sector observation must not revoke
        # a good cache, even if it carries a far-future timestamp.
        module.LookAtSoundServer.on_target(server, bearing(9999, 0.0, 1.0))
        module.LookAtSoundServer.on_target(server, bearing(9998, 0.5, 0.0))
        module.LookAtSoundServer.on_target(server, bearing(9997, float("nan")))
        self.assertIs(server.latest, first)
        self.assertIsNone(server.revoked_stamp)
        self.assertEqual(server.goal_evidence_count, 2)
        self.assertEqual(server.sequence, 1)

        second = bearing(11)
        module.LookAtSoundServer.on_target(server, second)
        self.assertIs(server.latest, second)
        self.assertEqual(server.sequence, 2)

        # Explicit invalidation still revokes, and delayed valid replay
        # cannot restore the invalidated observation.
        module.LookAtSoundServer.on_target(server, bearing(13, valid=False))
        self.assertIsNone(server.latest)
        self.assertEqual(server.revoked_stamp, (0, 13, 0))
        self.assertEqual(server.goal_evidence_count, 0)
        module.LookAtSoundServer.on_target(server, bearing(12))
        self.assertIsNone(server.latest)
        module.LookAtSoundServer.on_target(server, bearing(14))
        self.assertEqual(server.sequence, 3)

    def test_newer_invalidation_breaks_confirmation_streak(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            latest = None
            last_stamp = None
            sequence = 0
            arrival = 0.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0

        server = Server()
        self.target.header.stamp.sec = 10
        module.LookAtSoundServer.on_target(server, self.target)
        server.goal_evidence = self.target
        server.goal_evidence_count = 2
        server.goal_evidence_sequence = server.sequence

        stale = AttentionTarget()
        stale.header.frame_id = "base_link"
        stale.header.stamp.sec = 9
        module.LookAtSoundServer.on_target(server, stale)
        self.assertEqual(server.goal_evidence_count, 2)
        self.assertIs(server.latest, self.target)

        revoked = AttentionTarget()
        revoked.header.frame_id = "base_link"
        revoked.header.stamp.sec = 11
        module.LookAtSoundServer.on_target(server, revoked)
        self.assertIsNone(server.latest)
        self.assertIsNone(server.goal_evidence)
        self.assertEqual(server.goal_evidence_count, 0)
        self.assertEqual(server.goal_evidence_sequence, 1)

        recovered = AttentionTarget()
        recovered.valid = True
        recovered.header.frame_id = "base_link"
        recovered.direction_unit.x = 1.0
        recovered.header.stamp.sec = 12
        module.LookAtSoundServer.on_target(server, recovered)
        self.assertEqual(server.sequence, 2)
        count, anchor = module.accumulate_bearing_evidence(
            server.goal_evidence_count, server.goal_evidence, recovered, 0.2)
        self.assertEqual(count, 1)
        self.assertIs(anchor, recovered)

    def test_atomic_candidate_evaluation_after_revocation(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            max_age = 0.5
            confirmation_angle = 0.2
            confirmations = 3
            latest = None
            last_stamp = None
            sequence = 0
            arrival = 0.0
            goal_start_sequence = 0
            goal_stamp = None
            goal_deadline = 100.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0

            evaluate_candidate_locked = module.LookAtSoundServer.evaluate_candidate_locked

        server = Server()
        with patch.object(module.time, "monotonic", return_value=10.0):
            for stamp in (10, 11):
                observation = AttentionTarget()
                observation.valid = True
                observation.source_id = "voice-a"
                observation.header.frame_id = "base_link"
                observation.header.stamp.sec = stamp
                observation.direction_unit.x = 1.0
                module.LookAtSoundServer.on_target(server, observation)
                with server.lock:
                    self.assertIsNone(server.evaluate_candidate_locked(10.1, "voice-a"))
            self.assertEqual(server.goal_evidence_count, 2)
            # This is the stale worker snapshot that used to be read before
            # acquiring the lock, then counted after an invalidation callback.
            old_candidate = server.latest
            revoked = AttentionTarget()
            revoked.header.frame_id = "base_link"
            revoked.header.stamp.sec = 12
            module.LookAtSoundServer.on_target(server, revoked)
            self.assertIsNone(server.latest)
            with server.lock:
                self.assertIsNone(server.evaluate_candidate_locked(10.1, "voice-a"))
            self.assertEqual(server.goal_evidence_count, 0)
            self.assertIsNotNone(old_candidate)
            for stamp in (13, 14, 15):
                observation = AttentionTarget()
                observation.valid = True
                observation.source_id = "voice-a"
                observation.header.frame_id = "base_link"
                observation.header.stamp.sec = stamp
                observation.direction_unit.x = 1.0
                module.LookAtSoundServer.on_target(server, observation)
                with server.lock:
                    result = server.evaluate_candidate_locked(10.1, "voice-a")
                self.assertEqual(result is not None, stamp == 15)

    def test_confirmation_gap_resets_stale_partial_streak(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            max_age = 0.5
            confirmation_angle = 0.0
            confirmation_gap = 0.15
            confirmations = 3
            latest = None
            last_stamp = None
            revoked_stamp = None
            sequence = 0
            arrival = 0.0
            goal_start_sequence = 0
            goal_stamp = None
            goal_deadline = 100.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0
            goal_evidence_arrival = None
            evaluate_candidate_locked = module.LookAtSoundServer.evaluate_candidate_locked

        server = Server()
        def deliver(stamp, arrival):
            msg = AttentionTarget()
            msg.valid = True
            msg.source_id = "voice-a"
            msg.header.frame_id = "base_link"
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = 1.0
            with patch.object(module.time, "monotonic", return_value=arrival):
                module.LookAtSoundServer.on_target(server, msg)
            with server.lock:
                return server.evaluate_candidate_locked(arrival + 0.01, "voice-a")

        self.assertIsNone(deliver(1, 10.0))
        self.assertIsNone(deliver(2, 10.1))
        self.assertEqual(server.goal_evidence_count, 2)
        self.assertIsNone(deliver(3, 10.4))
        self.assertEqual(server.goal_evidence_count, 1)
        self.assertIsNone(deliver(4, 10.5))
        self.assertIsNotNone(deliver(5, 10.6))
        self.assertEqual(server.goal_evidence_count, 3)

    def test_confirmation_total_window_restarts_even_with_short_gaps(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            max_age = 0.5
            confirmation_angle = 0.0
            confirmation_gap = 0.4
            confirmation_window = 0.5
            confirmations = 3
            latest = None
            last_stamp = None
            revoked_stamp = None
            sequence = 0
            arrival = 0.0
            goal_start_sequence = 0
            goal_stamp = None
            goal_deadline = 100.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0
            goal_evidence_arrival = None
            goal_evidence_start_arrival = None
            evaluate_candidate_locked = module.LookAtSoundServer.evaluate_candidate_locked

        server = Server()

        def deliver(stamp, arrival):
            msg = AttentionTarget()
            msg.valid = True
            msg.source_id = "voice-a"
            msg.header.frame_id = "base_link"
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = 1.0
            with patch.object(module.time, "monotonic", return_value=arrival):
                module.LookAtSoundServer.on_target(server, msg)
            with server.lock:
                return server.evaluate_candidate_locked(arrival + 0.01, "")

        self.assertIsNone(deliver(10, 10.0))
        self.assertIsNone(deliver(11, 10.3))
        # Both adjacent gaps fit, but the whole run exceeds 500 ms.
        self.assertIsNone(deliver(12, 10.6))
        self.assertEqual(server.goal_evidence_count, 1)
        self.assertAlmostEqual(server.goal_evidence_start_arrival, 10.6)
        self.assertIsNone(deliver(13, 10.8))
        self.assertIsNotNone(deliver(14, 11.0))
        self.assertEqual(server.goal_evidence_count, 3)

    def test_minimum_confirmation_spacing_rejects_bursts(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            max_age = 0.5
            confirmation_angle = 0.0
            confirmation_gap = 0.0
            confirmation_window = 0.0
            confirmation_spacing = 0.2
            confirmations = 3
            latest = None
            last_stamp = None
            revoked_stamp = None
            sequence = 0
            arrival = 0.0
            goal_start_sequence = 0
            goal_stamp = None
            goal_deadline = 100.0
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0
            goal_evidence_arrival = None
            goal_evidence_start_arrival = None
            evaluate_candidate_locked = module.LookAtSoundServer.evaluate_candidate_locked

        server = Server()

        def deliver(stamp, arrival):
            msg = AttentionTarget()
            msg.valid = True
            msg.source_id = "voice-a"
            msg.header.frame_id = "base_link"
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = 1.0
            with patch.object(module.time, "monotonic", return_value=arrival):
                module.LookAtSoundServer.on_target(server, msg)
            with server.lock:
                return server.evaluate_candidate_locked(arrival + 0.01, "voice-a")

        self.assertIsNone(deliver(10, 10.0))
        self.assertIsNone(deliver(11, 10.05))
        self.assertIsNone(deliver(12, 10.10))
        self.assertEqual(server.goal_evidence_count, 1)
        self.assertIsNone(deliver(13, 10.21))
        self.assertEqual(server.goal_evidence_count, 2)
        self.assertIsNone(deliver(14, 10.25))
        self.assertEqual(server.goal_evidence_count, 2)
        self.assertIsNotNone(deliver(15, 10.42))
        self.assertEqual(server.goal_evidence_count, 3)

    def test_atomic_candidate_does_not_accept_expired_deadline(self):
        import threading

        class Server:
            lock = threading.Lock()
            frame = "base_link"
            max_azimuth = 0.0
            max_elevation = 0.0
            max_age = 0.5
            confirmation_angle = 0.0
            confirmations = 1
            latest = None
            last_stamp = None
            sequence = 0
            arrival = 0.0
            goal_start_sequence = 0
            goal_stamp = None
            goal_deadline = 10.05
            goal_evidence = None
            goal_evidence_count = 0
            goal_evidence_sequence = 0

            evaluate_candidate_locked = module.LookAtSoundServer.evaluate_candidate_locked

        server = Server()
        self.target.header.stamp.sec = 10
        with patch.object(module.time, "monotonic", return_value=10.0):
            module.LookAtSoundServer.on_target(server, self.target)
        with server.lock:
            self.assertIsNone(server.evaluate_candidate_locked(10.1, "voice-a"))
            self.assertEqual(server.goal_evidence_count, 0)

    def test_admission_deadline_is_not_execution_deadline(self):
        # A delayed action worker must not extend the accepted timeout.
        class Request:
            timeout_ms = 200
        class Server:
            reserved = False
            max_timeout = 10000
            sequence = 0
            last_stamp = None
            lock = __import__("threading").Lock()
        server = Server()
        with patch.object(module.time, "monotonic", return_value=10.0):
            response = module.LookAtSoundServer.on_goal(server, Request())
        self.assertEqual(response, module.GoalResponse.ACCEPT)
        self.assertAlmostEqual(server.goal_deadline, 10.2)

    def test_optional_bearing_sector(self):
        self.target.header.stamp.sec = 10
        self.target.direction_unit.x = -1.0
        self.assertTrue(module.admissible_observation(self.target, "base_link"))
        self.assertFalse(module.admissible_observation(self.target, "base_link", math.pi / 2, 0.0))
        self.target.direction_unit.x = 0.0
        self.target.direction_unit.y = 1.0
        self.assertTrue(module.admissible_observation(self.target, "base_link", math.pi / 2, 0.0))
        self.assertFalse(module.admissible_observation(self.target, "base_link", 0.5, 0.0))
        self.target.direction_unit.y = 0.0
        self.target.direction_unit.z = 1.0
        self.assertFalse(module.admissible_observation(self.target, "base_link", 0.0, 0.5))
        self.assertTrue(module.admissible_observation(self.target, "base_link", 0.0, math.pi / 2))

    def test_future_arrival_and_nonfinite_coordinates(self):
        self.assertFalse(self.check(now=0.9, arrival=1.0))
        self.assertFalse(self.check(now=float("nan")))
        self.target.direction_unit.y = float("inf")
        self.assertFalse(self.check())
        self.target.direction_unit.y = 0.0
        self.target.direction_unit.z = float("-inf")
        self.assertFalse(self.check())

    def test_sector_boundaries_and_outside(self):
        self.target.direction_unit.x = math.cos(0.6)
        self.target.direction_unit.y = math.sin(0.6)
        self.assertTrue(module.admissible_observation(self.target, "base_link", 0.6, 0.0))
        self.assertFalse(module.admissible_observation(self.target, "base_link", 0.59, 0.0))
        self.target.direction_unit.x = math.cos(0.4)
        self.target.direction_unit.y = 0.0
        self.target.direction_unit.z = math.sin(0.4)
        self.assertTrue(module.admissible_observation(self.target, "base_link", 0.0, 0.4))
        self.assertFalse(module.admissible_observation(self.target, "base_link", 0.0, 0.39))

    def test_stale(self):
        self.assertFalse(self.check(now=1.6))

    def test_source_and_frame(self):
        self.assertFalse(self.check(source="voice-b"))
        self.assertFalse(self.check(frame="map"))

    def test_invalid_and_nonunit(self):
        self.target.valid = False
        self.assertFalse(self.check())
        self.target.valid = True
        self.target.direction_unit.x = float("nan")
        self.assertFalse(self.check())
        self.target.direction_unit.x = 0.5
        self.assertFalse(self.check())


if __name__ == "__main__":
    unittest.main()
