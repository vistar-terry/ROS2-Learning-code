"""
⑩ 机器人多臂协调 —— 多执行器隔离不同子系统的实时性
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='robot_multi_arm',
            name='robot_multi_arm',
            output='screen',
            emulate_tty=True,
        ),
    ])
