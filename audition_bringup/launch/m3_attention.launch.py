"""Explicit, opt-in bearing-only attention consumer. No motion actions."""
import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configure(context, *args, **kwargs):
    spec = importlib.util.spec_from_file_location(
        "audition_m3_attention_config", Path(__file__).with_name("attention_config.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parameters = LaunchConfiguration("attention_parameters").perform(context)
    result = module.resolve_attention("true", parameters)
    namespace = LaunchConfiguration("audition_namespace").perform(context)
    # Reuse the established namespace validator, including rejecting leading '/'.
    ns_spec = importlib.util.spec_from_file_location(
        "audition_m3_profile_config", Path(__file__).with_name("profile_config.py"))
    ns_module = importlib.util.module_from_spec(ns_spec)
    import sys
    sys.modules[ns_spec.name] = ns_module
    ns_spec.loader.exec_module(ns_module)
    ns_module.validate_namespace(namespace)
    return [Node(
        package="audition_behaviors",
        executable="attention_manager",
        name="attention_manager",
        namespace=namespace,
        parameters=[result.parameters_path] if result.parameters_path else [],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("audition_namespace", default_value="audition"),
        DeclareLaunchArgument("attention_parameters", default_value=""),
        OpaqueFunction(function=_configure),
    ])
