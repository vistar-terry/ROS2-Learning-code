"""
launch/robot_demo.launch.py
机器人TF树演示启动文件

启动方式：
    ros2 launch tf2_learning robot_demo.launch.py

功能：启动机器人TF树广播器，并用RViz可视化
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取RViz配置文件路径
    pkg_dir = get_package_share_directory('tf2_learning')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robot_tf.rviz')

    # 机器人TF树广播器节点
    robot_tf_node = Node(
        package='tf2_learning',
        executable='robot_tf_broadcaster',
        name='robot_tf_broadcaster',
        output='screen',
    )

    # RViz2可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_tf_node,
        rviz_node,
    ])
