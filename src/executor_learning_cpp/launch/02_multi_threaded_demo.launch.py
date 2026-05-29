"""
② 多线程执行器 —— 并发回调与线程安全
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='multi_threaded_demo',
            name='multi_threaded_demo',
            output='screen',
            emulate_tty=True,
        ),
    ])
