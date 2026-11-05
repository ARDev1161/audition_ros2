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
        Node(package="audition_behaviors", executable="look_at_sound",
             namespace=LaunchConfiguration("audition_namespace"),
             parameters=[{
                 "target_frame": LaunchConfiguration("target_frame"),
                 "maximum_target_age_ms": ParameterValue(
                     LaunchConfiguration("maximum_target_age_ms"), value_type=int),
                 "maximum_goal_timeout_ms": ParameterValue(
                     LaunchConfiguration("maximum_goal_timeout_ms"), value_type=int),
             }], output="screen"),
    ])
