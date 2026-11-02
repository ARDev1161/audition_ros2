"""M2 semantic pipeline atop the independently isolated M1 WAV/ODAS capture."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
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
        DeclareLaunchArgument("classifier_backend", default_value="disabled"),
        DeclareLaunchArgument("yamnet_model", default_value=""),
        DeclareLaunchArgument("yamnet_labels", default_value=""),
        DeclareLaunchArgument("window_samples", default_value="15600"),
        DeclareLaunchArgument("hop_samples", default_value="7800"),
        DeclareLaunchArgument("event_start_threshold", default_value="0.70"),
        DeclareLaunchArgument("event_stop_threshold", default_value="0.40"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                "wav_path": LaunchConfiguration("wav_path"),
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
                    parameters=[{
                        "classifier_backend": ParameterValue(
                            LaunchConfiguration("classifier_backend"), value_type=str),
                        "yamnet_model": ParameterValue(
                            LaunchConfiguration("yamnet_model"), value_type=str),
                        "yamnet_labels": ParameterValue(
                            LaunchConfiguration("yamnet_labels"), value_type=str),
                        "window_samples": LaunchConfiguration("window_samples"),
                        "hop_samples": LaunchConfiguration("hop_samples"),
                        "event_start_threshold": LaunchConfiguration("event_start_threshold"),
                        "event_stop_threshold": LaunchConfiguration("event_stop_threshold"),
                    }],
                ),
            ],
        ),
    ])
