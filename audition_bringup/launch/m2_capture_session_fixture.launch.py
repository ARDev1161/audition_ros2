"""CI-only graph fixture: mono and canonical scene without a capture publisher."""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name="capture_session_test_container",
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
                ComposableNode(
                    package="audition_perception",
                    plugin="audition_perception::AcousticSceneNode",
                    name="acoustic_scene",
                    namespace="/audition",
                ),
            ],
        ),
    ])
