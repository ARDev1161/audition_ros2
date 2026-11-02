"""Live ALSA + ODAS + optional YAMNet/Sherpa/Whisper semantic processing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = PathJoinSubstitution([
        FindPackageShare("audition_bringup"), "launch", "m1_alsa_pipeline.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("device", description="ALSA capture PCM name"),
        DeclareLaunchArgument("frame_id", default_value="microphone_array"),
        DeclareLaunchArgument("sample_rate_hz", default_value="16000"),
        DeclareLaunchArgument("channel_count", default_value="6"),
        DeclareLaunchArgument("frames_per_message", default_value="160"),
        DeclareLaunchArgument("spatial_backend", default_value="disabled"),
        DeclareLaunchArgument(
            "spatial_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("audition_bringup"), "config", "respeaker_usb_v2.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "semantic_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("audition_bringup"), "config", "semantic_m2.yaml"
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                "device": LaunchConfiguration("device"),
                "frame_id": LaunchConfiguration("frame_id"),
                "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
                "channel_count": LaunchConfiguration("channel_count"),
                "frames_per_message": LaunchConfiguration("frames_per_message"),
                "spatial_backend": LaunchConfiguration("spatial_backend"),
                "spatial_config": LaunchConfiguration("spatial_config"),
            }.items(),
        ),
        ComposableNodeContainer(
            name="audition_semantic_container",
            namespace="audition",
            package="rclcpp_components",
            executable="component_container",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="audition_perception",
                    plugin="audition_perception::SemanticPerceptionNode",
                    namespace="/audition",
                    name="semantic_perception",
                    parameters=[LaunchConfiguration("semantic_config")],
                ),
            ],
        ),
    ])
