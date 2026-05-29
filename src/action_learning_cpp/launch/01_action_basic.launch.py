"""① Action 基础：Server + Client

演示最基本的 Action 通信：
  Server 接受目标 → 执行 → 返回结果
  Client 发送目标 → 等待结果

用法：
  ros2 launch action_learning_cpp 01_action_basic.launch.py
  ros2 launch action_learning_cpp 01_action_basic.launch.py target:=20
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 可配置参数：目标值
        DeclareLaunchArgument(
            'target', default_value='10',
            description='计数目标值'),
        # Action Server
        Node(
            package='action_learning_cpp',
            executable='01_action_server_basic',
            name='count_up_server',
            output='screen',
            emulate_tty=True,
        ),
        # Action Client（延迟2秒启动，等 Server 就绪）
        Node(
            package='action_learning_cpp',
            executable='02_action_client_basic',
            name='count_up_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target': LaunchConfiguration('target'),
            }],
        ),
    ])
