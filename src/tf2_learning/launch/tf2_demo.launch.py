"""
launch/tf2_demo.launch.py
TF2学习包的统一启动文件

启动方式：
    ros2 launch tf2_learning tf2_demo.launch.py

功能：同时启动动态广播器和监听器，演示TF2的基本用法
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明launch参数：TF发布频率（Hz）
    tf_frequency_arg = DeclareLaunchArgument(
        'tf_frequency',
        default_value='20.0',
        description='TF发布频率（Hz）'
    )

    # 动态TF广播器节点
    dynamic_broadcaster_node = Node(
        package='tf2_learning',
        executable='dynamic_tf_broadcaster',
        name='dynamic_tf_broadcaster',
        output='screen',
    )

    # 静态TF广播器节点
    static_broadcaster_node = Node(
        package='tf2_learning',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen',
    )

    # TF监听器节点
    tf_listener_node = Node(
        package='tf2_learning',
        executable='tf_listener',
        name='tf_listener',
        output='screen',
    )

    return LaunchDescription([
        tf_frequency_arg,
        dynamic_broadcaster_node,
        static_broadcaster_node,
        tf_listener_node,
    ])
