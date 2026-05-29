"""
⑧ 执行器与回调组配合 —— 回调组如何影响执行器调度
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='executor_callback_groups',
            name='executor_callback_groups',
            output='screen',
            emulate_tty=True,
        ),
    ])
