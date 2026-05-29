#!/usr/bin/env python3
"""
Launch文件 ⑥ —— 机器人传感器融合
运行方式: ros2 launch callback_group_learning 06_robot_sensor_fusion.launch.py

注意：此演示需要传感器话题 (/scan, /imu/data, /camera/image_raw)
可以使用模拟传感器数据源，或配合 Gazebo 仿真使用
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='callback_group_learning',
            executable='robot_sensor_fusion',
            name='robot_sensor_fusion_node',
            output='screen',
            # 可以在此重映射传感器话题
            # remappings=[
            #     ('/scan', '/lidar/scan'),
            #     ('/imu/data', '/imu/raw'),
            # ],
        ),
    ])
