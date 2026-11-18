"""M4-A diagnostic-only integrated graph for one namespaced acoustic scene.

This launch composes existing M4 adapters; it does not start capture, ODAS,
models, physical navigation, or any metric localization publisher.
Input topics: sources, vision/bearings and fusion/input_pose.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns = LaunchConfiguration('audition_namespace')
    world = LaunchConfiguration('world_frame')
    microphone = LaunchConfiguration('microphone_frame')
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace', default_value='audition'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('microphone_frame', default_value='microphone_array'),
        DeclareLaunchArgument('pose_type', default_value='odometry'),
        DeclareLaunchArgument('require_tf_consistency', default_value='false'),
        DeclareLaunchArgument('enable_conditional_evidence', default_value='false'),
        DeclareLaunchArgument('microphone_extrinsics_calibrated',
                              default_value='false'),
        Node(package='audition_fusion', executable='av_pair_node',
             namespace=ns, name='av_pair_node',
             parameters=[{'world_frame': world}], output='screen'),
        Node(package='audition_fusion', executable='bearing_candidate_node',
             namespace=ns, name='bearing_candidate_node',
             parameters=[{'world_frame': world}], output='screen'),
        Node(package='audition_fusion', executable='pose_evidence_node',
             namespace=ns, name='pose_evidence_node',
             parameters=[{
                 'world_frame': world,
                 'microphone_frame': microphone,
                 'pose_type': LaunchConfiguration('pose_type'),
                 'require_tf_consistency': ParameterValue(
                     LaunchConfiguration('require_tf_consistency'),
                     value_type=bool),
                 'enable_conditional_evidence': ParameterValue(
                     LaunchConfiguration('enable_conditional_evidence'),
                     value_type=bool),
                 'microphone_extrinsics_calibrated': ParameterValue(
                     LaunchConfiguration('microphone_extrinsics_calibrated'),
                     value_type=bool),
                 'maximum_pose_age_ms': 500,
                 'maximum_pose_skew_ms': 50,
                 'pose_buffer_capacity': 32,
                 'microphone_lever_arm_xy_m': [0.0, 0.0],
             }], output='screen'),
    ])
