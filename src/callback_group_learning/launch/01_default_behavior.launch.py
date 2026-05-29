#!/usr/bin/env python3
"""
Launch文件 ① —— 默认行为演示
运行方式: ros2 launch callback_group_learning 01_default_behavior.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='default_behavior',
            name='default_behavior_node',
            output='screen',
        ),
    ])
