"""Opt-in camera pan/tilt proposals; never publishes joint or motion commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace', default_value='audition'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('maximum_pan_rad', default_value='1.2'),
        DeclareLaunchArgument('maximum_tilt_rad', default_value='0.7'),
        DeclareLaunchArgument('maximum_age_ms', default_value='500'),
        Node(package='audition_behaviors', executable='attention_camera_aim',
             namespace=LaunchConfiguration('audition_namespace'),
             parameters=[{
                 'target_frame': LaunchConfiguration('target_frame'),
                 'maximum_pan_rad': ParameterValue(LaunchConfiguration('maximum_pan_rad'), value_type=float),
                 'maximum_tilt_rad': ParameterValue(LaunchConfiguration('maximum_tilt_rad'), value_type=float),
                 'maximum_age_ms': ParameterValue(LaunchConfiguration('maximum_age_ms'), value_type=int),
             }]),
    ])
