"""Opt-in M4 diagnostic bearing intersections, NOT metric localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace', default_value='audition'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('minimum_baseline_m', default_value='0.5'),
        DeclareLaunchArgument('minimum_parallax_sine', default_value='0.15'),
        DeclareLaunchArgument('maximum_ray_residual_m', default_value='0.25'),
        DeclareLaunchArgument('maximum_observation_age_ms', default_value='750'),
        Node(package='audition_fusion', executable='bearing_candidate_node',
             namespace=LaunchConfiguration('audition_namespace'),
             parameters=[{
                 'world_frame': LaunchConfiguration('world_frame'),
                 'minimum_baseline_m': ParameterValue(
                     LaunchConfiguration('minimum_baseline_m'), value_type=float),
                 'minimum_parallax_sine': ParameterValue(
                     LaunchConfiguration('minimum_parallax_sine'), value_type=float),
                 'maximum_ray_residual_m': ParameterValue(
                     LaunchConfiguration('maximum_ray_residual_m'), value_type=float),
                 'maximum_observation_age_ms': ParameterValue(
                     LaunchConfiguration('maximum_observation_age_ms'), value_type=int),
             }],
             output='screen'),
    ])
