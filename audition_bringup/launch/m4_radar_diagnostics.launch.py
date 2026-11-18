"""M4-D opt-in radar + acoustic diagnostic DDS graph (no localization)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns = LaunchConfiguration('audition_namespace')
    world = LaunchConfiguration('world_frame')
    age = LaunchConfiguration('maximum_observation_age_ms')
    return LaunchDescription([
        DeclareLaunchArgument('audition_namespace', default_value='audition'),
        DeclareLaunchArgument('world_frame', default_value='odom'),
        DeclareLaunchArgument('maximum_observation_age_ms', default_value='800'),
        Node(package='audition_fusion', executable='radar_adapter_node',
             namespace=ns, parameters=[{
                 'world_frame': world,
                 'maximum_observation_age_ms': ParameterValue(age,value_type=int),
             }], output='screen'),
        Node(package='audition_fusion', executable='radar_audio_association_node',
             namespace=ns, parameters=[{
                 'world_frame': world,
                 'maximum_observation_age_ms': ParameterValue(age,value_type=int),
             }], output='screen'),
    ])
