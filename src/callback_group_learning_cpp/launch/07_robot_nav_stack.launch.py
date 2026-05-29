#!/usr/bin/env python3
"""Launch文件 ⑦ —— 机器人导航栈 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='robot_nav_stack',
            name='robot_nav_stack_node',
            output='screen',
        ),
    ])
