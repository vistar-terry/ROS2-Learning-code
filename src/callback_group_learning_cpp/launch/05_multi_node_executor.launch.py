#!/usr/bin/env python3
"""Launch文件 ⑤ —— 多节点单执行器 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='multi_node_executor',
            name='multi_node_executor_demo',
            output='screen',
        ),
    ])
