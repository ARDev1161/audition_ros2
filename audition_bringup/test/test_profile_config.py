"""M2c fail-closed profile validation regressions (no ROS or model binaries)."""
import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import wave

import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("profile_config", ROOT / "launch" / "profile_config.py")
profile = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = profile
SPEC.loader.exec_module(profile)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.wav = self.root / "mono.wav"
        self.make_wav(1, 16000)
        self.base = {
            "schema_version": 1,
            "name": "test_profile",
            "capture": {
                "kind": "wav", "wav_path": str(self.wav),
                "frame_id": "mic", "channels": 1, "sample_rate_hz": 16000,
            },
            "spatial": {"backend": "mono"},
            "semantic": {"classifier_backend": "disabled", "speech_enabled": False},
        }
        self.path = self.root / "profile.yaml"

    def make_wav(self, channels, rate):
        with wave.open(str(self.wav), "wb") as stream:
            stream.setnchannels(channels)
            stream.setsampwidth(2)
            stream.setframerate(rate)
            stream.writeframes(struct.pack("<" + "h" * channels * 8, *([0] * channels * 8)))

    def load(self):
        self.path.write_text(yaml.safe_dump(self.base), encoding="utf-8")
        return profile.resolve_profile(str(self.path), ROOT)

    def rejects(self, part):
        with self.assertRaisesRegex(profile.ProfileError, part):
            self.load()

    def test_builtin_replay(self):
        resolved = profile.resolve_profile("replay_ci", ROOT, {"wav_path": str(self.wav)})
        self.assertEqual(resolved.spatial_backend, "mono")
        self.assertEqual(resolved.semantic["profile_name"], "replay_ci")
        self.assertFalse(resolved.semantic["speech_enabled"])
        self.assertEqual(resolved.capture["channels"], 1)

    def test_missing_wav_is_rejected(self):
        self.base["capture"]["wav_path"] = "/not/a/real/file.wav"
        self.rejects("capture.wav_path")

    def test_wrong_wav_rate_is_rejected(self):
        self.make_wav(1, 8000)
        self.rejects("disagrees with WAV")

    def test_wrong_wav_channel_count_is_rejected(self):
        self.make_wav(2, 16000)
        self.rejects("disagrees with WAV")

    def test_mono_requires_one_channel(self):
        self.make_wav(2, 16000)
        self.base["capture"]["channels"] = 2
        self.rejects("mono requires")

    def test_unknown_backend_fails_closed(self):
        self.base["spatial"]["backend"] = "guess"
        self.rejects("spatial.backend")

    def test_unknown_semantic_option_is_rejected(self):
        self.base["semantic"]["asr_magic"] = True
        self.rejects("unknown keys")

    def test_threshold_nan_and_bool_are_rejected(self):
        self.base["semantic"]["event_start_threshold"] = float("nan")
        self.rejects("finite")
        self.base["semantic"]["event_start_threshold"] = True
        self.rejects("finite")

    def test_speech_requires_models(self):
        self.base["semantic"]["speech_enabled"] = True
        self.rejects("speech_vad_model")

    def test_classifier_requires_models(self):
        self.base["semantic"]["classifier_backend"] = "yamnet"
        self.rejects("yamnet_model")

    def test_explicit_model_dir_and_no_auto_selection(self):
        model_dir = self.root / "models"
        model_dir.mkdir()
        for name in ("tagger.onnx", "labels.txt"):
            (model_dir / name).write_bytes(b"test")
        self.base["models_dir"] = str(model_dir)
        self.base["semantic"].update({
            "classifier_backend": "yamnet",
            "yamnet_model": "tagger.onnx",
            "yamnet_labels": "labels.txt",
        })
        resolved = self.load()
        self.assertEqual(resolved.semantic["yamnet_model"], str(model_dir / "tagger.onnx"))
        self.assertEqual(len(profile.discover_models(model_dir)), 2)

    def test_relative_model_path_cannot_escape_directory(self):
        model_dir = self.root / "models"
        model_dir.mkdir()
        (self.root / "outside.onnx").write_bytes(b"test")
        self.base["models_dir"] = str(model_dir)
        self.base["semantic"].update({
            "classifier_backend": "yamnet",
            "yamnet_model": "../outside.onnx",
            "yamnet_labels": "labels.txt",
        })
        self.rejects("escapes models_dir")

    def test_no_silent_semantic_without_source_audio(self):
        self.base["spatial"]["backend"] = "disabled"
        self.base["semantic"]["speech_enabled"] = True
        self.rejects("Speech requires mono or ODAS")

    def test_odas_uncalibrated_is_rejected(self):
        self.make_wav(6, 16000)
        self.base["capture"]["channels"] = 6
        self.base["spatial"] = {
            "backend": "odas",
            "config": str(ROOT / "config" / "respeaker_usb_v2.yaml"),
        }
        self.rejects("measured geometry_calibrated")

    def test_odas_validated_geometry_and_frame(self):
        self.make_wav(6, 16000)
        self.base["capture"]["channels"] = 6
        geometry = self.root / "geometry.yaml"
        cfg = {"spatial_perception": {"ros__parameters": {
            "geometry_calibrated": True, "microphone_frame": "mic",
            "sample_rate_hz": 16000, "input_channel_indices": [1, 2, 3, 4],
            "microphone_positions_m": [0, 0, 0, 0.1, 0, 0,
                                       0, 0.1, 0, 0.1, 0.1, 0],
        }}}
        geometry.write_text(yaml.safe_dump(cfg), encoding="utf-8")
        self.base["spatial"] = {"backend": "odas", "config": str(geometry)}
        self.assertEqual(self.load().spatial_backend, "odas")
        cfg["spatial_perception"]["ros__parameters"]["microphone_frame"] = "wrong"
        geometry.write_text(yaml.safe_dump(cfg), encoding="utf-8")
        self.rejects("must match capture.frame_id")

    def test_alsa_requires_explicit_device(self):
        self.base["capture"] = {
            "kind": "alsa", "device": "", "channels": 1,
            "sample_rate_hz": 16000, "frame_id": "mic",
        }
        self.rejects("ALSA profile requires")
        resolved = profile.resolve_profile(
            self._write(), ROOT, {"device": "hw:1,0"})
        self.assertEqual(resolved.capture["device"], "hw:1,0")

    def _write(self):
        self.path.write_text(yaml.safe_dump(self.base), encoding="utf-8")
        return str(self.path)


if __name__ == "__main__":
    unittest.main()
