from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='config.yaml'),
        Node(
            package='calibration_manager',
            executable='calibration_manager',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='delay_estimator',
            executable='delay_estimator',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='feature_tracker',
            executable='feature_tracker',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='horizon_detection',
            executable='horizon_detection',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='stream_telemetry',
            executable='stream_telemetry',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='telemetry_synchronizer',
            executable='telemetry_synchronizer',
            parameters=[params_file],
            output='screen',
        ),
    ])
