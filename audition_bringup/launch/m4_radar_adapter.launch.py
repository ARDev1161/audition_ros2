"""Opt-in M4-C radar track TF2 diagnostics; no Nav2 or motion outputs."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace', default_value='audition'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('maximum_observation_age_ms', default_value='800'),
        DeclareLaunchArgument('maximum_tracks', default_value='16'),
        Node(package='audition_fusion', executable='radar_adapter_node',
             namespace=LaunchConfiguration('audition_namespace'),
             parameters=[{
                 'world_frame': LaunchConfiguration('world_frame'),
                 'maximum_observation_age_ms': ParameterValue(
                     LaunchConfiguration('maximum_observation_age_ms'),
                     value_type=int),
                 'maximum_tracks': ParameterValue(
                     LaunchConfiguration('maximum_tracks'), value_type=int),
             }], output='screen'),
    ])
