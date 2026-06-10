"""Python 日志系统演示 —— 独立进程方式启动"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error, fatal'
    )

    return LaunchDescription([
        log_level_arg,
        Node(
            package='logging_learning_py',
            executable='logging_basic',
            name='logging_basic',
            output='screen',
        ),
        Node(
            package='logging_learning_py',
            executable='logging_levels',
            name='logging_levels',
            output='screen',
            parameters=[{
                'log_level': LaunchConfiguration('log_level'),
            }],
        ),
        Node(
            package='logging_learning_py',
            executable='logging_advanced',
            name='logging_advanced',
            output='screen',
        ),
        Node(
            package='logging_learning_py',
            executable='logging_robot_demo',
            name='robot_logging_demo',
            output='screen',
        ),
    ])
