"""M2 semantic pipeline; configure models and thresholds via semantic_config YAML."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base = PathJoinSubstitution([
        FindPackageShare("audition_bringup"), "launch", "m1_wav_pipeline.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("wav_path", description="Absolute mono/multichannel WAV"),
        DeclareLaunchArgument("spatial_backend", default_value="disabled"),
        DeclareLaunchArgument(
            "spatial_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("audition_bringup"), "config", "respeaker_usb_v2.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "semantic_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("audition_bringup"), "config", "semantic_m2.yaml",
            ]),
            description="Full semantic backend/model/threshold ROS parameters YAML",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                "wav_path": LaunchConfiguration("wav_path"),
                "spatial_backend": LaunchConfiguration("spatial_backend"),
                "spatial_config": LaunchConfiguration("spatial_config"),
            }.items(),
        ),
        # Isolate model inference from spatial processing even when the native
        # libaudition backend is enabled. Its worker is independently bounded.
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
