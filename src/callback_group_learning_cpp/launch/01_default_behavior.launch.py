#!/usr/bin/env python3
"""Launch文件 ① —— 默认行为演示 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='default_behavior',
            name='default_behavior_node',
            output='screen',
        ),
    ])
