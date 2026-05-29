#!/usr/bin/env python3
"""
Launch文件 ⑧ —— 死锁演示与解决
运行方式:
  死锁演示:   ros2 launch callback_group_learning 08_deadlock_demo.launch.py mode:=1
  正确版本:   ros2 launch callback_group_learning 08_deadlock_demo.launch.py mode:=2
  最佳实践:   ros2 launch callback_group_learning 08_deadlock_demo.launch.py mode:=3
  默认(mode=3): 最佳实践
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明模式参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='3',
        description='演示模式: 1=死锁, 2=正确版本, 3=最佳实践(默认)',
    )

    return LaunchDescription([
        mode_arg,
        Node(
            package='callback_group_learning',
            executable='deadlock_demo',
            name='deadlock_demo_node',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
            }],
            # 通过命令行参数传递模式
            # 注意：这里用 arguments 而非 parameters，因为节点用 sys.argv 解析
        ),
    ])
