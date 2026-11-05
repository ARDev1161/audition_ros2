#!/usr/bin/env python3
"""Pure policy regression for LookAtSound freshness and source matching."""
import importlib.util
import pathlib
import unittest

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

    def test_fresh(self):
        self.assertTrue(self.check())
        self.assertTrue(self.check(source="voice-a"))

    def test_goal_requires_new_observation(self):
        self.assertFalse(module.fresh_target(self.target, 1.0, 1.1, 0.5, "", "base_link", 5, 5))
        self.assertFalse(module.fresh_target(self.target, 1.0, 1.1, 0.5, "", "base_link", 4, 5))
        self.assertTrue(module.fresh_target(self.target, 1.0, 1.1, 0.5, "", "base_link", 6, 5))

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
