#!/usr/bin/env python3
"""
Launch文件 ⑦ —— 机器人导航栈
运行方式: ros2 launch callback_group_learning 07_robot_nav_stack.launch.py

注意：此演示需要 /scan 话题
可以配合 Gazebo + diff_robot_description 包使用
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='robot_nav_stack',
            name='robot_nav_stack_node',
            output='screen',
        ),
    ])
