#!/usr/bin/env python3
"""
Launch文件 ④ —— 执行器类型对比
运行方式: ros2 launch callback_group_learning 04_executor_types.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='executor_types',
            name='executor_test_node',
            output='screen',
        ),
    ])
