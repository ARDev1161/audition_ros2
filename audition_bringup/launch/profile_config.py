"""Fail-closed M2c startup profile resolver. No ROS import: usable in CI/preflight.

A profile is startup-only. The resolver never guesses a model file or
substitutes a synthetic array for an uncalibrated physical microphone.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import re
import struct

import yaml


class ProfileError(ValueError):
    """Invalid or unsupported pipeline configuration."""


@dataclass(frozen=True)
class ResolvedProfile:
    name: str
    capture: dict
    spatial_backend: str
    spatial_config: str
    scene_config: str
    semantic: dict


def _mapping(value, where):
    if not isinstance(value, dict):
        raise ProfileError(f"{where} must be a YAML mapping")
    return value


def _keys(data, allowed, where):
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ProfileError(f"{where}: unknown keys {unknown}")


def _file(path, description):
    p = Path(path).expanduser().resolve()
    if not p.is_file():
        raise ProfileError(f"{description} is not a regular file: {p}")
    return str(p)


def _config_file(value, base, description):
    p = Path(value).expanduser()
    if not p.is_absolute():
        p = base / p
    return _file(p, description)


def _wav_format(path):
    """Read the RIFF fmt header without loading audio or rejecting float32 WAV."""
    with open(path, "rb") as stream:
        if stream.read(4) != b"RIFF" or len(stream.read(4)) != 4 or stream.read(4) != b"WAVE":
            raise ProfileError("capture.wav_path is not a RIFF/WAVE file")
        for _ in range(64):
            header = stream.read(8)
            if len(header) != 8:
                break
            tag, size = struct.unpack("<4sI", header)
            if tag == b"fmt ":
                if size < 16:
                    break
                fmt = stream.read(16)
                if len(fmt) != 16:
                    break
                encoding, channels, rate, _, _, bits = struct.unpack("<HHIIHH", fmt)
                if (encoding, bits) not in ((1, 16), (3, 32)):
                    raise ProfileError("WAV must be PCM16 or IEEE float32")
                return channels, rate
            stream.seek(size + (size % 2), 1)
    raise ProfileError("capture.wav_path has no valid RIFF fmt chunk")


def _model(value, model_dir, name):
    if not isinstance(value, str) or not value:
        raise ProfileError(f"{name} requires an explicit model filename")
    p = Path(value).expanduser()
    if not p.is_absolute():
        if model_dir is None:
            raise ProfileError(f"{name} is relative; supply models_dir")
        p = (model_dir / p).resolve()
        if not p.is_relative_to(model_dir):
            raise ProfileError(f"{name} escapes models_dir")
    return _file(p, name)


def _int(value, minimum, maximum, name):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ProfileError(f"{name} must be an integer in [{minimum}, {maximum}]")


def _real(value, minimum, maximum, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ProfileError(f"{name} must be finite and in [{minimum}, {maximum}]")


def _spatial_geometry(path, channels, frame):
    with open(path, encoding="utf-8") as stream:
        data = _mapping(yaml.safe_load(stream), "spatial configuration")
    node = data.get("spatial_perception", data.get("/**/spatial_perception"))
    node = _mapping(node, "spatial_perception")
    p = _mapping(node.get("ros__parameters"), "spatial_perception.ros__parameters")
    if p.get("geometry_calibrated") is not True:
        raise ProfileError("ODAS requires measured geometry_calibrated: true")
    if p.get("microphone_frame") != frame:
        raise ProfileError("ODAS microphone_frame must match capture.frame_id")
    if p.get("sample_rate_hz") != 16000:
        raise ProfileError("ODAS spatial sample rate must be 16000")
    indices = p.get("input_channel_indices")
    xyz = p.get("microphone_positions_m")
    if (not isinstance(indices, list) or len(indices) < 4 or
            any(type(i) is not int or i < 0 or i >= channels for i in indices) or
            len(set(indices)) != len(indices)):
        raise ProfileError("ODAS input_channel_indices must be unique valid capture channels (>=4)")
    if (not isinstance(xyz, list) or len(xyz) != 3 * len(indices) or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in xyz)):
        raise ProfileError("ODAS microphone_positions_m must contain finite xyz triples")
    points = [xyz[i:i+3] for i in range(0, len(xyz), 3)]
    if len({tuple(v) for v in points}) != len(points):
        raise ProfileError("ODAS microphone positions must be distinct")
    # At least three non-collinear microphones. This is necessary, not a
    # substitute for physical calibration or a real DOA accuracy test.
    a = points[0]
    vectors = [[p[i]-a[i] for i in range(3)] for p in points[1:]]
    cross = lambda x, y: (x[1]*y[2]-x[2]*y[1],
                           x[2]*y[0]-x[0]*y[2],
                           x[0]*y[1]-x[1]*y[0])
    if not any(sum(c*c for c in cross(x, y)) > 1e-12
               for x in vectors for y in vectors):
        raise ProfileError("ODAS microphone positions are collinear")


def discover_models(directory):
    """List candidate model files; never auto-select by ambiguous filename."""
    base = Path(directory).expanduser().resolve()
    if not base.is_dir():
        raise ProfileError(f"models_dir does not exist: {base}")
    extensions = {".onnx", ".gguf", ".bin", ".txt"}
    return sorted(str(p) for p in base.iterdir()
                  if p.is_file() and p.suffix.lower() in extensions)[:128]


def resolve_profile(profile, package_share, overrides=None):
    """Validate a named preset or explicit YAML path; return ROS launch arguments."""
    overrides = overrides or {}
    _keys(overrides, {"wav_path", "device", "models_dir"}, "launch overrides")
    share = Path(package_share).resolve()
    if re.fullmatch(r"[a-z][a-z0-9_]*", profile):
        path = share / "config" / "profiles" / (profile + ".yaml")
    else:
        path = Path(profile).expanduser().resolve()
    path = Path(_file(path, "profile"))
    with open(path, encoding="utf-8") as stream:
        data = _mapping(yaml.safe_load(stream), "profile")
    _keys(data, {"schema_version", "name", "capture", "spatial", "semantic",
                 "models_dir", "scene_config"}, "profile")
    if type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ProfileError("Unsupported profile schema_version (expected 1)")
    name = data.get("name")
    if not isinstance(name, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", name):
        raise ProfileError("profile.name must be a short lowercase identifier")
    capture = _mapping(data.get("capture"), "capture").copy()
    _keys(capture, {"kind", "wav_path", "device", "frame_id", "sample_rate_hz",
                    "channels", "frames_per_message"}, "capture")
    kind = capture.get("kind")
    if kind not in ("wav", "alsa"):
        raise ProfileError("capture.kind must be wav or alsa")
    frame = capture.get("frame_id", "microphone_array")
    if not isinstance(frame, str) or not frame or frame.startswith("/"):
        raise ProfileError("capture.frame_id must be a nonempty relative TF frame")
    capture["frame_id"] = frame
    _int(capture.get("frames_per_message", 160), 1, 4096, "frames_per_message")
    capture.setdefault("frames_per_message", 160)
    for key in ("wav_path", "device"):
        if overrides.get(key):
            capture[key] = overrides[key]
    if kind == "wav":
        wav = capture.get("wav_path", "")
        if not isinstance(wav, str) or not wav:
            raise ProfileError("WAV profile requires wav_path:=/absolute/file.wav")
        wav = _config_file(wav, path.parent, "capture.wav_path")
        capture["wav_path"] = wav
        channels, rate = _wav_format(wav)
        if "channels" in capture and capture["channels"] != channels:
            raise ProfileError("capture.channels disagrees with WAV fmt header")
        if "sample_rate_hz" in capture and capture["sample_rate_hz"] != rate:
            raise ProfileError("capture.sample_rate_hz disagrees with WAV fmt header")
        capture["channels"], capture["sample_rate_hz"] = channels, rate
    else:
        if not isinstance(capture.get("device"), str) or not capture["device"]:
            raise ProfileError("ALSA profile requires device:=<PCM name>")
        _int(capture.get("channels"), 1, 32, "capture.channels")
        _int(capture.get("sample_rate_hz"), 8000, 192000, "capture.sample_rate_hz")
    spatial = _mapping(data.get("spatial"), "spatial")
    _keys(spatial, {"backend", "config"}, "spatial")
    backend = spatial.get("backend")
    if backend not in ("mono", "odas", "disabled"):
        raise ProfileError("spatial.backend must be mono, odas or disabled")
    if backend == "mono" and (capture["channels"] != 1 or capture["sample_rate_hz"] != 16000):
        raise ProfileError("mono requires exactly one 16 kHz capture channel")
    if backend == "odas" and (capture["channels"] < 4 or capture["sample_rate_hz"] != 16000):
        raise ProfileError("ODAS requires at least four 16 kHz capture channels")
    spatial_config = str(share / "config" / "respeaker_usb_v2.yaml")
    if backend == "odas":
        if not spatial.get("config"):
            raise ProfileError("ODAS requires spatial.config with measured geometry")
        spatial_config = _config_file(spatial["config"], path.parent, "spatial.config")
        _spatial_geometry(spatial_config, capture["channels"], frame)
    elif spatial.get("config"):
        raise ProfileError("spatial.config is only accepted with backend: odas")

    scene_config = str(share / "config" / "acoustic_scene.yaml")
    if data.get("scene_config"):
        scene_config = _config_file(data["scene_config"], path.parent, "scene_config")

    with open(share / "config" / "semantic_m2.yaml", encoding="utf-8") as stream:
        defaults = yaml.safe_load(stream)["/**/semantic_perception"]["ros__parameters"]
    semantic = _mapping(data.get("semantic", {}), "semantic")
    _keys(semantic, defaults.keys(), "semantic")
    params = {**defaults, **semantic}
    classifier = params["classifier_backend"]
    if classifier not in ("disabled", "yamnet"):
        raise ProfileError("Unsupported classifier_backend: " + str(classifier))
    if type(params["speech_enabled"]) is not bool:
        raise ProfileError("speech_enabled must be boolean")
    for key, lo, hi in (("yamnet_top_k", 1, 521), ("yamnet_threads", 1, 32),
                        ("window_samples", 1, 160000), ("hop_samples", 1, 160000),
                        ("max_sources", 1, 128), ("event_min_start_observations", 1, 100),
                        ("event_min_stop_observations", 1, 100),
                        ("speech_asr_threads", 1, 32), ("speech_max_sources", 1, 64),
                        ("speech_max_pending_segments", 1, 128),
                        ("speech_native_initial_context_ms", 0, 1000)):
        _int(params[key], lo, hi, key)
    if params["hop_samples"] > params["window_samples"]:
        raise ProfileError("hop_samples must not exceed window_samples")
    for key in ("event_start_threshold", "event_stop_threshold", "speech_vad_threshold"):
        _real(params[key], 0.0, 1.0, key)
    if params["event_start_threshold"] < params["event_stop_threshold"]:
        raise ProfileError("event_start_threshold must be >= event_stop_threshold")
    if not 0 < params["speech_vad_threshold"] < 1:
        raise ProfileError("speech_vad_threshold must be strictly between 0 and 1")
    if type(params["speech_normalize_asr_segments"]) is not bool:
        raise ProfileError("speech_normalize_asr_segments must be boolean")
    if classifier == "yamnet" and capture["sample_rate_hz"] != 16000:
        raise ProfileError("YAMNet requires 16 kHz capture")
    if params["speech_enabled"] and capture["sample_rate_hz"] != 16000:
        raise ProfileError("Speech requires 16 kHz capture")
    if classifier != "disabled" and backend == "disabled":
        raise ProfileError("Classifier requires mono or ODAS source audio")
    if params["speech_enabled"] and backend == "disabled":
        raise ProfileError("Speech requires mono or ODAS source audio")

    models_dir = overrides.get("models_dir") or data.get("models_dir", "")
    base = None
    if models_dir:
        base = Path(models_dir).expanduser()
        if not base.is_absolute():
            base = path.parent / base
        base = base.resolve()
        if not base.is_dir():
            raise ProfileError(f"models_dir is not a directory: {base}")
    if classifier == "yamnet":
        for key in ("yamnet_model", "yamnet_labels"):
            params[key] = _model(params[key], base, key)
    if params["speech_enabled"]:
        if params["speech_vad_backend"] != "sherpa" or params["speech_asr_backend"] != "whispercpp":
            raise ProfileError("Only sherpa VAD + whispercpp ASR are implemented")
        for key in ("speech_vad_model", "speech_asr_model"):
            params[key] = _model(params[key], base, key)
        if not isinstance(params["speech_language"], str) or not params["speech_language"]:
            raise ProfileError("speech_language must be nonempty")
    params["profile_name"] = name
    params["profile_spatial_backend"] = backend
    return ResolvedProfile(name, capture, backend, spatial_config, scene_config, params)
