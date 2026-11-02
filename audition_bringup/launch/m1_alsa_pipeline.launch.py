"""Live ALSA capture + spatial perception + canonical scene (experimental M1)."""

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "device",
            description="Explicit ALSA capture PCM (e.g. hw:2,0, plughw:2,0, or CI-only null)",
        ),
        DeclareLaunchArgument("frame_id", default_value="microphone_array"),
        DeclareLaunchArgument("sample_rate_hz", default_value="16000"),
        DeclareLaunchArgument("channel_count", default_value="6"),
        DeclareLaunchArgument("frames_per_message", default_value="160"),
        DeclareLaunchArgument("spatial_backend", default_value="disabled"),
        DeclareLaunchArgument("scene_config", default_value=PathJoinSubstitution([FindPackageShare("audition_bringup"), "config", "acoustic_scene.yaml"])),
        DeclareLaunchArgument(
            "spatial_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("audition_bringup"), "config", "respeaker_usb_v2.yaml",
            ]),
        ),
        Node(
            package="audition_capture",
            executable="alsa_capture",
            namespace="audition",
            name="audio_capture",
            output="screen",
            parameters=[{
                "device": ParameterValue(LaunchConfiguration("device"), value_type=str),
                "frame_id": ParameterValue(LaunchConfiguration("frame_id"), value_type=str),
                "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
                "channel_count": LaunchConfiguration("channel_count"),
                "frames_per_message": LaunchConfiguration("frames_per_message"),
            }],
        ),
        ComposableNodeContainer(
            condition=UnlessCondition(PythonExpression([
                "'", LaunchConfiguration("spatial_backend"), "' == 'mono'"
            ])),
            name="audition_spatial_container",
            namespace="audition",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="audition_perception",
                    plugin="audition_perception::SpatialPerceptionNode",
                    name="spatial_perception",
                    namespace="/audition",
                    parameters=[
                        LaunchConfiguration("spatial_config"),
                        {"backend": LaunchConfiguration("spatial_backend")},
                    ],
                ),
            ],
        ),
        # Dedicated mono semantic route, not a spatial localization backend.
        ComposableNodeContainer(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("spatial_backend"), "' == 'mono'"
            ])),
            name="audition_mono_container",
            namespace="audition",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="audition_perception",
                    plugin="audition_perception::MonoSourceNode",
                    name="mono_source",
                    namespace="/audition",
                ),
            ],
        ),
        ComposableNodeContainer(
            name="audition_scene_container",
            namespace="audition",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="audition_perception",
                    plugin="audition_perception::AcousticSceneNode",
                    name="acoustic_scene",
                    namespace="/audition",
                    parameters=[LaunchConfiguration("scene_config")],
                ),
            ],
        ),
    ])
