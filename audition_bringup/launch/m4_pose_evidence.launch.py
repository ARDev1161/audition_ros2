"""Opt-in M4 pose/DOA timestamp diagnostic; no metric localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace', default_value='audition'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('pose_type', default_value='odometry'),
        DeclareLaunchArgument('require_tf_consistency', default_value='false'),
        DeclareLaunchArgument('microphone_frame', default_value='microphone_array'),
        DeclareLaunchArgument('enable_conditional_evidence', default_value='false'),
        DeclareLaunchArgument('microphone_extrinsics_calibrated', default_value='false'),
        Node(
            package='audition_fusion',
            executable='pose_evidence_node',
            namespace=LaunchConfiguration('audition_namespace'),
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
                'pose_type': LaunchConfiguration('pose_type'),
                'require_tf_consistency': ParameterValue(LaunchConfiguration('require_tf_consistency'), value_type=bool),
                'microphone_frame': LaunchConfiguration('microphone_frame'),
                'enable_conditional_evidence': ParameterValue(LaunchConfiguration('enable_conditional_evidence'), value_type=bool),
                'microphone_extrinsics_calibrated': ParameterValue(LaunchConfiguration('microphone_extrinsics_calibrated'), value_type=bool),
                'maximum_pose_age_ms': 500,
                'maximum_pose_skew_ms': 50,
                'pose_buffer_capacity': 32,
                # Diagnostic-only default. Specify measured extrinsics before
                # interpreting any position uncertainty physically.
                'microphone_lever_arm_xy_m': [0.0, 0.0],
            }],
            output='screen')
    ])
