"""M1 contract-validation pipeline: real WAV PCM, no acoustic localization yet."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    wav_path = LaunchConfiguration("wav_path")
    return LaunchDescription([
        DeclareLaunchArgument("wav_path", description="Absolute path to a PCM16/float32 RIFF WAV"),
        DeclareLaunchArgument("frame_id", default_value="microphone_array"),
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
        # Isolated process boundary allows ODAS backend integration later.
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
                    parameters=[{"backend": "disabled"}],
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
