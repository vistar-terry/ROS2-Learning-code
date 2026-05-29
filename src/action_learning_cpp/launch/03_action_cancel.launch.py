"""③ Action 取消演示

演示：
  发送一个长时间目标，3秒后自动取消
  Server 优雅处理取消（清理资源、返回中间结果）
  自包含：Server + Client 在同一进程

用法：
  ros2 launch action_learning_cpp 03_action_cancel.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_learning_cpp',
            executable='05_action_cancel_demo',
            name='cancel_demo',
            output='screen',
            emulate_tty=True,
        ),
    ])
