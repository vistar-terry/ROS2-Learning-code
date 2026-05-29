#!/usr/bin/env python3
"""
Launch文件 ② —— MutuallyExclusiveCallbackGroup 演示
运行方式: ros2 launch callback_group_learning 02_mutually_exclusive.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='mutually_exclusive_demo',
            name='mutually_exclusive_demo_node',
            output='screen',
        ),
    ])
