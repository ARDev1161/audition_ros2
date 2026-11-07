"""Opt-in timestamped TF2 conversion of advisory attention bearings; no motion."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("audition_namespace", default_value="audition"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("maximum_age_ms", default_value="500"),
        Node(package="audition_behaviors", executable="attention_tf_bridge",
             namespace=LaunchConfiguration("audition_namespace"),
             parameters=[{
                 "target_frame": LaunchConfiguration("target_frame"),
                 "maximum_age_ms": ParameterValue(LaunchConfiguration("maximum_age_ms"), value_type=int),
             }],
             output="screen"),
    ])
