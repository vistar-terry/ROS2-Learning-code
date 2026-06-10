"""日志系统演示 —— 通过 launch 参数配置日志级别"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明日志级别参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error, fatal'
    )

    return LaunchDescription([
        log_level_arg,
        # ① 基础日志演示
        Node(
            package='logging_learning_cpp',
            executable='logging_basic',
            name='logging_basic',
            output='screen',
        ),
        # ② 日志级别控制演示（通过参数设置级别）
        Node(
            package='logging_learning_cpp',
            executable='logging_levels',
            name='logging_levels',
            output='screen',
            parameters=[{
                'log_level': LaunchConfiguration('log_level'),
            }],
        ),
        # ③ 高级日志演示
        Node(
            package='logging_learning_cpp',
            executable='logging_advanced',
            name='logging_advanced',
            output='screen',
        ),
        # ⑤ 机器人实战案例
        Node(
            package='logging_learning_cpp',
            executable='logging_robot_demo',
            name='robot_logging_demo',
            output='screen',
        ),
    ])
