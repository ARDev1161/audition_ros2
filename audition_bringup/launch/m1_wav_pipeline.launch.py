"""M1 replay: real WAV input, optional native libaudition ODAS spatial processing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wav_path = LaunchConfiguration("wav_path")
    spatial_config = LaunchConfiguration("spatial_config")
    spatial_backend = LaunchConfiguration("spatial_backend")
    return LaunchDescription([
        DeclareLaunchArgument("wav_path", description="Absolute PCM16 or float32 RIFF WAV path"),
        DeclareLaunchArgument("frame_id", default_value="microphone_array"),
        DeclareLaunchArgument("spatial_backend", default_value="disabled",
                              description="disabled (safe default) or odas (requires calibrated geometry)"),
        DeclareLaunchArgument(
            "spatial_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("audition_bringup"), "config", "respeaker_usb_v2.yaml"]),
            description="ROS parameters YAML with microphone geometry for spatial_perception"),
        Node(
            package="audition_capture",
            executable="wav_capture",
            namespace="audition",
            name="audio_capture",
            parameters=[{
                "wav_path": wav_path,
                "frame_id": LaunchConfiguration("frame_id"),
                "frames_per_message": 160,
                "loop": False,
            }],
            output="screen",
        ),
        ComposableNodeContainer(
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
                    parameters=[spatial_config, {"backend": spatial_backend}],
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
                ),
            ],
        ),
    ])
