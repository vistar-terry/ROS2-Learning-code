#!/usr/bin/env python3
"""Launch文件 ③ —— Reentrant回调组演示 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='reentrant_demo',
            name='reentrant_demo_node',
            output='screen',
        ),
    ])
