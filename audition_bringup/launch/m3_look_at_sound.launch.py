"""Opt-in LookAtSound bearing acquisition action; no robot motion."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("audition_namespace", default_value="audition"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("maximum_target_age_ms", default_value="500"),
        DeclareLaunchArgument("maximum_goal_timeout_ms", default_value="10000"),
        DeclareLaunchArgument("maximum_abs_azimuth_rad", default_value="0.0"),
        DeclareLaunchArgument("maximum_abs_elevation_rad", default_value="0.0"),
        DeclareLaunchArgument("bearing_confirmations", default_value="1"),
        DeclareLaunchArgument("maximum_confirmation_jump_rad", default_value="0.0"),
        DeclareLaunchArgument("minimum_confirmation_spacing_ms", default_value="0"),
        DeclareLaunchArgument("maximum_confirmation_gap_ms", default_value="0"),
        DeclareLaunchArgument("maximum_confirmation_window_ms", default_value="0"),
        Node(package="audition_behaviors", executable="look_at_sound",
             namespace=LaunchConfiguration("audition_namespace"),
             parameters=[{
                 "target_frame": LaunchConfiguration("target_frame"),
                 "bearing_confirmations": ParameterValue(LaunchConfiguration("bearing_confirmations"), value_type=int),
                 "maximum_confirmation_jump_rad": ParameterValue(LaunchConfiguration("maximum_confirmation_jump_rad"), value_type=float),
                 "minimum_confirmation_spacing_ms": ParameterValue(LaunchConfiguration("minimum_confirmation_spacing_ms"), value_type=int),
                 "maximum_confirmation_gap_ms": ParameterValue(LaunchConfiguration("maximum_confirmation_gap_ms"), value_type=int),
                 "maximum_confirmation_window_ms": ParameterValue(LaunchConfiguration("maximum_confirmation_window_ms"), value_type=int),
                 "maximum_abs_azimuth_rad": ParameterValue(LaunchConfiguration("maximum_abs_azimuth_rad"), value_type=float),
                 "maximum_abs_elevation_rad": ParameterValue(LaunchConfiguration("maximum_abs_elevation_rad"), value_type=float),
                 "maximum_target_age_ms": ParameterValue(
                     LaunchConfiguration("maximum_target_age_ms"), value_type=int),
                 "maximum_goal_timeout_ms": ParameterValue(
                     LaunchConfiguration("maximum_goal_timeout_ms"), value_type=int),
             }], output="screen"),
    ])
