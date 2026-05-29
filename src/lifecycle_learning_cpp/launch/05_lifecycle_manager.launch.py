"""⑤ 外部状态管理器 —— 通过服务调用管理生命周期节点

本 launch 文件同时启动被管理节点和 manager，开箱即用。
也可手动分两终端运行（适合调试）：
  终端1: ros2 launch lifecycle_learning_cpp 01_lifecycle_basic.launch.py
  终端2: ros2 run lifecycle_learning_cpp lifecycle_manager_node
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 被管理的生命周期节点
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='lifecycle_basic',
            name='lifecycle_basic',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
        # 外部管理器
        Node(
            package='lifecycle_learning_cpp',
            executable='lifecycle_manager_node',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
        ),
    ])
