"""M3 attention launch validation; no ROS runtime required."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml

spec = importlib.util.spec_from_file_location(
    "attention_config", Path(__file__).resolve().parents[1] / "launch" / "attention_config.py")
config = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = config
spec.loader.exec_module(config)


class AttentionConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "attention.yaml"

    def write(self, params):
        self.path.write_text(yaml.safe_dump({
            "/**/attention_manager": {"ros__parameters": params}
        }), encoding="utf-8")
        return str(self.path)

    def test_disabled_and_opt_in_defaults(self):
        self.assertFalse(config.resolve_attention("false", "").enabled)
        self.assertTrue(config.resolve_attention("true", "").enabled)
        self.assertEqual(config.resolve_attention("true", "").parameters_path, "")

    def test_valid_parameters(self):
        path = self.write({"minimum_activity": 0.4, "switch_confirmations": 3,
                           "maximum_source_age_ms": 500,
                           "require_advancing_scene_stamp": True})
        result = config.resolve_attention("true", path)
        self.assertEqual(result.parameters_path, path)

    def test_reject_ambiguous_or_disabled_configuration(self):
        for flag in ("TRUE", "1", "", "yes"):
            with self.assertRaises(config.AttentionConfigError):
                config.resolve_attention(flag, "")
        with self.assertRaises(config.AttentionConfigError):
            config.resolve_attention("false", self.write({"minimum_activity": 0.3}))

    def test_reject_unknown_and_invalid_parameters(self):
        for params in ({"unknown": 1}, {"switch_confirmations": 0},
                       {"switch_confirmations": True},
                       {"maximum_direction_jump_rad": 4.0},
                       {"minimum_activity": float("nan")},
                       {"scene_timeout_ms": -1},
                       {"require_advancing_scene_stamp": 1},
                       {"require_advancing_scene_stamp": "true"},
                       {"require_advancing_scene_stamp": None}):
            with self.subTest(params=params):
                with self.assertRaises(config.AttentionConfigError):
                    config.resolve_attention("true", self.write(params))

    def test_reject_invalid_path_and_schema(self):
        with self.assertRaises(config.AttentionConfigError):
            config.resolve_attention("true", "relative.yaml")
        with self.assertRaises(config.AttentionConfigError):
            config.resolve_attention("true", str(self.path))
        self.path.write_text(yaml.safe_dump({"attention_manager": {
            "ros__parameters": {"minimum_activity": 0.5}}}), encoding="utf-8")
        with self.assertRaises(config.AttentionConfigError):
            config.resolve_attention("true", str(self.path))

    def test_strict_replay_bool_accepts_both_values(self):
        for flag in (True, False):
            with self.subTest(flag=flag):
                path = self.write({"require_advancing_scene_stamp": flag})
                self.assertEqual(config.resolve_attention("true", path).parameters_path, path)
