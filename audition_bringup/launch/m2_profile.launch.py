"""Validated M2c startup profiles for WAV/ALSA + mono/ODAS + semantics.

This launch is opt-in; legacy M1/M2 launch files remain supported. Preflight
resolves all paths and rejects invalid profiles before starting any nodes.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _resolver():
    spec = importlib.util.spec_from_file_location(
        "audition_m2_profile_config", Path(__file__).with_name("profile_config.py"))
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves its module through sys.modules
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _configure(context, *args, **kwargs):
    share = Path(get_package_share_directory("audition_bringup"))
    value = lambda key: LaunchConfiguration(key).perform(context)
    resolver = _resolver()
    namespace = resolver.validate_namespace(value("audition_namespace"))
    profile = resolver.resolve_profile(value("profile"), share, {
        "wav_path": value("wav_path"),
        "device": value("device"),
        "models_dir": value("models_dir"),
        "frame_id": value("frame_id"),
    })
    capture = profile.capture
    if namespace != "audition" and capture["frame_id"] == "microphone_array":
        raise resolver.ProfileError("Non-default namespace requires unique frame_id:=...")
    kind = capture["kind"]
    base = share / "launch" / ("m1_wav_pipeline.launch.py" if kind == "wav"
                               else "m1_alsa_pipeline.launch.py")
    arguments = {
        "audition_namespace": namespace,
        "spatial_backend": profile.spatial_backend,
        "spatial_config": profile.spatial_config,
        "scene_config": profile.scene_config,
        "frame_id": capture["frame_id"],
        "frames_per_message": str(capture["frames_per_message"]),
    }
    if kind == "wav":
        arguments["wav_path"] = capture["wav_path"]
    else:
        arguments.update({
            "device": capture["device"],
            "sample_rate_hz": str(capture["sample_rate_hz"]),
            "channel_count": str(capture["channels"]),
        })
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(base)),
                                 launch_arguments=arguments.items()),
        ComposableNodeContainer(
            name="audition_semantic_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="audition_perception",
                    plugin="audition_perception::SemanticPerceptionNode",
                    namespace="/" + namespace,
                    name="semantic_perception",
                    parameters=[profile.semantic],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("audition_namespace", default_value="audition"),
        DeclareLaunchArgument("frame_id", default_value="",
                              description="Explicit unique TF frame for additional sensors"),
        DeclareLaunchArgument("profile", default_value="mono_speech",
                              description="Built-in profile name or absolute YAML path"),
        DeclareLaunchArgument("wav_path", default_value="",
                              description="Override WAV file in profile"),
        DeclareLaunchArgument("device", default_value="",
                              description="Override ALSA PCM device in profile"),
        DeclareLaunchArgument("models_dir", default_value="",
                              description="Directory for explicitly named model files"),
        OpaqueFunction(function=_configure),
    ])
