"""
⑥ 多节点单执行器 —— 一个执行器管理多个节点

用法：
  ros2 launch executor_learning_cpp 06_multi_node_single_executor.launch.py mode:=1  # MultiThreaded
  ros2 launch executor_learning_cpp 06_multi_node_single_executor.launch.py mode:=2  # SingleThreaded
  ros2 launch executor_learning_cpp 06_multi_node_single_executor.launch.py mode:=3  # 动态添加/移除
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='1',
        description='模式: 1=MultiThreaded, 2=SingleThreaded, 3=动态添加/移除'
    )

    multi_node = Node(
        package='executor_learning_cpp',
        executable='multi_node_single_executor',
        name='multi_node_single_executor',
        output='screen',
        emulate_tty=True,
        arguments=[LaunchConfiguration('mode')],
    )

    return LaunchDescription([mode_arg, multi_node])
