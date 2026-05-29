#!/usr/bin/env python3
"""
Launch文件 ③ —— ReentrantCallbackGroup 演示
运行方式: ros2 launch callback_group_learning 03_reentrant.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='reentrant_demo',
            name='reentrant_demo_node',
            output='screen',
        ),
    ])
