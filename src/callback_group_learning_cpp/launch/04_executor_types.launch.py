#!/usr/bin/env python3
"""Launch文件 ④ —— 执行器类型对比 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='executor_types',
            name='executor_test_node',
            output='screen',
        ),
    ])
