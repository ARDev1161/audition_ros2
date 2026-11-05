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
        invalid.header.stamp.sec = 1000
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
        stale_invalid.header.stamp.sec = 19
        module.LookAtSoundServer.on_target(server, stale_invalid)
        self.assertIs(server.latest, newer)
        current_invalid = AttentionTarget()
        current_invalid.valid = False
        current_invalid.header.stamp.sec = 22
        module.LookAtSoundServer.on_target(server, current_invalid)
        self.assertIsNone(server.latest)
        self.assertEqual(server.last_stamp, (0, 21, 0))
        recovery = AttentionTarget()
        recovery.valid = True
        recovery.header.frame_id = "base_link"
        recovery.direction_unit.x = 1.0
        recovery.header.stamp.sec = 22
        module.LookAtSoundServer.on_target(server, recovery)
        self.assertIs(server.latest, recovery)
        self.assertEqual(server.sequence, 3)

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
