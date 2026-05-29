"""
③ 静态单线程执行器 —— 零分配高性能执行
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='static_executor',
            name='static_executor',
            output='screen',
            emulate_tty=True,
        ),
    ])
