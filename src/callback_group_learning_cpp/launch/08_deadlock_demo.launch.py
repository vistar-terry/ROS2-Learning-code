#!/usr/bin/env python3
"""
Launch文件 ⑧ —— 死锁演示与解决 (C++版)
运行方式:
  死锁演示:   ros2 launch callback_group_learning_cpp 08_deadlock_demo.launch.py mode:=1
  正确版本:   ros2 launch callback_group_learning_cpp 08_deadlock_demo.launch.py mode:=2
  最佳实践:   ros2 launch callback_group_learning_cpp 08_deadlock_demo.launch.py mode:=3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='3',
        description='演示模式: 1=死锁, 2=正确版本, 3=最佳实践(默认)',
    )

    return LaunchDescription([
        mode_arg,
        Node(
            package='callback_group_learning_cpp',
            executable='deadlock_demo',
            name='deadlock_demo_node',
            output='screen',
            # 通过命令行参数传递模式
            # C++ 节点通过 argv 解析
        ),
    ])
