"""
① 单线程执行器基础 —— 理解 rclcpp::spin() 背后的原理
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='single_threaded_basic',
            name='single_threaded_basic',
            output='screen',
            emulate_tty=True,
        ),
    ])
