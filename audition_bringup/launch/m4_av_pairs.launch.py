"""Opt-in M4 cross-sensor bearing pairing; never motion or localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace',default_value='audition'),
        DeclareLaunchArgument('world_frame',default_value='odom'),
        DeclareLaunchArgument('maximum_observation_age_ms',default_value='800'),
        DeclareLaunchArgument('maximum_skew_ms',default_value='150'),
        DeclareLaunchArgument('maximum_sensor_separation_m',default_value='0.2'),
        DeclareLaunchArgument('maximum_pair_angle_rad',default_value='0.30'),
        DeclareLaunchArgument('minimum_assumed_range_m',default_value='1.0'),
        Node(package='audition_fusion',executable='av_pair_node',
             namespace=LaunchConfiguration('audition_namespace'),
             parameters=[{
                 'world_frame':LaunchConfiguration('world_frame'),
                 'maximum_observation_age_ms':ParameterValue(LaunchConfiguration('maximum_observation_age_ms'),value_type=int),
                 'maximum_skew_ms':ParameterValue(LaunchConfiguration('maximum_skew_ms'),value_type=float),
                 'maximum_sensor_separation_m':ParameterValue(LaunchConfiguration('maximum_sensor_separation_m'),value_type=float),
                 'maximum_pair_angle_rad':ParameterValue(LaunchConfiguration('maximum_pair_angle_rad'),value_type=float),
                 'minimum_assumed_range_m':ParameterValue(LaunchConfiguration('minimum_assumed_range_m'),value_type=float),
             }],
             output='screen'),
    ])
