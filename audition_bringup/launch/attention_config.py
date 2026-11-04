"""Fail-closed, ROS-independent M3 attention launch preflight."""
from dataclasses import dataclass
from pathlib import Path
import math
import yaml


class AttentionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class AttentionLaunch:
    enabled: bool
    parameters_path: str


_ALLOWED = {
    "minimum_activity": (float, 0.0, 1.0),
    "switch_margin": (float, 0.0, 1.0),
    "scene_timeout_ms": (int, 1, 2147483647),
    "switch_confirmations": (int, 1, 100),
    "switch_confirmation_window_ms": (int, 1, 2147483647),
    "maximum_angular_variance_rad2": (float, 0.0, float("inf")),
    "maximum_source_age_ms": (int, 0, 2147483647),
    "maximum_direction_jump_rad": (float, 0.0, math.pi),
}


def resolve_attention(enabled, parameters_path):
    if enabled not in ("true", "false"):
        raise AttentionConfigError("enable_attention must be true or false")
    if not isinstance(parameters_path, str):
        raise AttentionConfigError("attention_parameters must be a path")
    if enabled == "false":
        if parameters_path:
            raise AttentionConfigError("attention_parameters requires enable_attention:=true")
        return AttentionLaunch(False, "")
    if not parameters_path:
        return AttentionLaunch(True, "")
    path = Path(parameters_path).expanduser()
    if not path.is_absolute() or not path.is_file():
        raise AttentionConfigError("attention_parameters must be an existing absolute YAML file")
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or set(data) != {"/**/attention_manager"}:
        raise AttentionConfigError("attention YAML must contain only /**/attention_manager")
    node = data["/**/attention_manager"]
    if not isinstance(node, dict) or set(node) != {"ros__parameters"}:
        raise AttentionConfigError("attention YAML requires ros__parameters")
    params = node["ros__parameters"]
    if not isinstance(params, dict) or not params:
        raise AttentionConfigError("attention ros__parameters must be nonempty")
    for key, value in params.items():
        if key not in _ALLOWED:
            raise AttentionConfigError("unknown attention parameter: " + str(key))
        kind, low, high = _ALLOWED[key]
        if (type(value) not in ((int,) if kind is int else (int, float)) or
                not math.isfinite(value) or not low <= value <= high):
            raise AttentionConfigError("invalid attention parameter: " + key)
    return AttentionLaunch(True, str(path.resolve()))
