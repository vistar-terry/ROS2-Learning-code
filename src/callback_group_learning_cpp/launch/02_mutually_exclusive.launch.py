#!/usr/bin/env python3
"""Launch文件 ② —— MutuallyExclusive回调组演示 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='mutually_exclusive_demo',
            name='mutually_exclusive_demo_node',
            output='screen',
        ),
    ])
