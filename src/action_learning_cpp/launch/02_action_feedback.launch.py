"""② Action 反馈：高频反馈 + 进度条

演示：
  Server: 10Hz 反馈、互斥执行、优雅取消
  Client: 进度条显示、ETA 估算

用法：
  ros2 launch action_learning_cpp 02_action_feedback.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target', default_value='5',
            description='计数目标值'),
        # Feedback Server
        Node(
            package='action_learning_cpp',
            executable='03_action_server_feedback',
            name='feedback_server',
            output='screen',
            emulate_tty=True,
        ),
        # Feedback Client
        Node(
            package='action_learning_cpp',
            executable='04_action_client_feedback',
            name='feedback_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target': LaunchConfiguration('target'),
            }],
        ),
    ])
