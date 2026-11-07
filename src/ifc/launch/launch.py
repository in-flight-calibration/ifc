from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='config.yaml'),
        Node(
            package='horizon_detection',
            executable='horizon_detection',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='feature_tracker',
            executable='feature_tracker',
            parameters=[params_file],
            output='screen',
        ),
    ])
