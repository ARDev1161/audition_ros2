"""Opt-in camera aim admission; still advisory, not an actuator driver."""
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
        DeclareLaunchArgument('authorization_timeout_ms', default_value='300'),
        Node(package='audition_behaviors', executable='attention_camera_admission',
             namespace=LaunchConfiguration('audition_namespace'),
             parameters=[{
                 'target_frame': LaunchConfiguration('target_frame'),
                 'maximum_pan_rad': ParameterValue(LaunchConfiguration('maximum_pan_rad'), value_type=float),
                 'maximum_tilt_rad': ParameterValue(LaunchConfiguration('maximum_tilt_rad'), value_type=float),
                 'authorization_timeout_ms': ParameterValue(LaunchConfiguration('authorization_timeout_ms'), value_type=int),
             }]),
    ])
