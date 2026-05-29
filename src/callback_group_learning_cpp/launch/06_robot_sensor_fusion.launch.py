#!/usr/bin/env python3
"""Launch文件 ⑥ —— 机器人传感器融合 (C++版)"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning_cpp',
            executable='robot_sensor_fusion',
            name='robot_sensor_fusion_node',
            output='screen',
        ),
    ])
