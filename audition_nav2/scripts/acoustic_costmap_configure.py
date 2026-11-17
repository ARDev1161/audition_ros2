#!/usr/bin/env python3
"""Safely add the optional acoustic overlay to an existing Nav2 parameter file.

The input is never modified. The generated file keeps existing obstacle,
inflation, footprint, and collision-related parameters intact.
"""
import argparse
from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit("python3-yaml is required") from exc

PLUGIN = "audition_nav2::AcousticCostmapLayer"


def configure(document, costmap, trusted_metric_topic=""):
    if not isinstance(document, dict):
        raise ValueError("Nav2 parameters must be a YAML mapping")
    # A topic is operator-approved input, never inferred from diagnostic M4 output.
    # Keep startup disabled even when a trusted topic is explicitly supplied.
    if not isinstance(trusted_metric_topic, str) or any(
            c.isspace() for c in trusted_metric_topic):
        raise ValueError("trusted metric topic must be a ROS topic name without whitespace")
    if trusted_metric_topic and (not trusted_metric_topic.startswith("/") or
                                 trusted_metric_topic == "/" or
                                 "//" in trusted_metric_topic or
                                 trusted_metric_topic.endswith("/") or
                                 "~" in trusted_metric_topic or
                                 "*" in trusted_metric_topic):
        raise ValueError("trusted metric topic must be an absolute ROS topic name")
    if costmap not in ("local_costmap", "global_costmap"):
        raise ValueError("unsupported costmap")
    outer = document.get(costmap)
    if not isinstance(outer, dict):
        raise ValueError("missing costmap node: " + costmap)
    inner = outer.get(costmap)
    if not isinstance(inner, dict):
        raise ValueError("missing nested costmap node: " + costmap)
    params = inner.get("ros__parameters")
    if not isinstance(params, dict):
        raise ValueError("missing ros__parameters")
    plugins = params.get("plugins")
    if not isinstance(plugins, list) or not plugins or any(
            not isinstance(name, str) or not name for name in plugins):
        raise ValueError("explicit nonempty plugins roster required")
    if len(set(plugins)) != len(plugins):
        raise ValueError("duplicate costmap plugins")
    if any(name == "acoustic_layer" for name in plugins[:-1]):
        raise ValueError("acoustic_layer must be terminal")
    if any(isinstance(v, dict) and v.get("plugin") == PLUGIN
           for k, v in params.items() if k != "acoustic_layer"):
        raise ValueError("acoustic plugin registered under another name")
    filters = params.get("filters", [])
    if not isinstance(filters, list):
        raise ValueError("filters must be a list")
    if "acoustic_layer" in filters:
        raise ValueError("acoustic layer must not be a filter")
    existing = params.get("acoustic_layer", {})
    if not isinstance(existing, dict):
        raise ValueError("acoustic_layer parameters must be a mapping")
    if existing.get("plugin", PLUGIN) != PLUGIN:
        raise ValueError("acoustic_layer name is already used by another plugin")
    if "acoustic_layer" not in plugins:
        plugins.append("acoustic_layer")
    # Fail closed even if the input file previously enabled acoustic costs.
    params["acoustic_layer"] = dict(existing, plugin=PLUGIN,
                                    enable_soft_costs=False,
                                    trusted_metric_topic=trusted_metric_topic)
    return document


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="existing Nav2 YAML")
    parser.add_argument("output", type=Path, help="new Nav2 YAML (must not exist)")
    parser.add_argument("--costmap", choices=("local_costmap", "global_costmap"),
                        default="local_costmap")
    parser.add_argument("--trusted-metric-topic", default="",
                        help="operator-approved absolute LocalizedSourceArray topic; "
                             "never inferred from diagnostic fusion output")
    args = parser.parse_args(argv)
    try:
        if args.input.resolve() == args.output.resolve():
            raise ValueError("input and output must differ")
        with args.input.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        document = configure(document, args.costmap, args.trusted_metric_topic)
        # Exclusive create prevents accidental overwrite of a working config.
        with args.output.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        print("Wrote disabled acoustic overlay to", args.output)
        print("Review the generated Nav2 parameters before using them.")
        return 0
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print("ERROR:", exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
