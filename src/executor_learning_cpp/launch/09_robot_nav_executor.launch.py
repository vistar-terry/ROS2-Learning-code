"""
⑨ 机器人导航栈 —— 执行器驱动的多优先级控制架构
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='robot_nav_executor',
            name='robot_nav_executor',
            output='screen',
            emulate_tty=True,
        ),
    ])
