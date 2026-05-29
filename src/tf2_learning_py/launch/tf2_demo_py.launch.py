"""
launch/tf2_demo_py.launch.py
TF2学习包Python版的统一启动文件

启动方式：
    ros2 launch tf2_learning_py tf2_demo_py.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 动态TF广播器节点
    dynamic_broadcaster_node = Node(
        package='tf2_learning_py',
        executable='dynamic_tf_broadcaster_py',
        name='dynamic_tf_broadcaster_py',
        output='screen',
    )

    # 静态TF广播器节点
    static_broadcaster_node = Node(
        package='tf2_learning_py',
        executable='static_tf_broadcaster_py',
        name='static_tf_broadcaster_py',
        output='screen',
    )

    # TF监听器节点
    tf_listener_node = Node(
        package='tf2_learning_py',
        executable='tf_listener_py',
        name='tf_listener_py',
        output='screen',
    )

    return LaunchDescription([
        dynamic_broadcaster_node,
        static_broadcaster_node,
        tf_listener_node,
    ])
