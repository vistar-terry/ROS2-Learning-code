#!/usr/bin/env python3
"""
Launch文件 ⑤ —— 多节点单执行器
运行方式: ros2 launch callback_group_learning 05_multi_node_executor.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='multi_node_executor',
            name='multi_node_executor_demo',
            output='screen',
        ),
    ])
