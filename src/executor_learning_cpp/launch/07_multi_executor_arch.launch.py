"""
⑦ 多执行器架构 —— 每个节点独立执行器 + 多线程
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='multi_executor_arch',
            name='multi_executor_arch',
            output='screen',
            emulate_tty=True,
        ),
    ])
