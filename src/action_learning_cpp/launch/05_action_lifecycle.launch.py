"""⑤ Lifecycle Action Server

演示：
  LifecycleNode 作为 Action Server
  按生命周期状态控制 Action 的接受/拒绝
  未激活时拒绝所有目标，激活后正常接受

需要手动触发生命周期转换：
  终端1: ros2 launch action_learning_cpp 05_action_lifecycle.launch.py
  终端2: ros2 lifecycle set /lifecycle_action_server configure
  终端2: ros2 lifecycle set /lifecycle_action_server activate
  （激活后 Client 的目标才会被接受）

  也可以用 CLI 测试：
  终端2: ros2 action send_goal /count_up action_learning_cpp/action/CountUp "{target: 5}"
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lifecycle Action Server
        LifecycleNode(
            package='action_learning_cpp',
            executable='07_action_lifecycle_server',
            name='lifecycle_action_server',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
        # Client（延迟启动，需要先手动 activate）
        Node(
            package='action_learning_cpp',
            executable='02_action_client_basic',
            name='lifecycle_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target': 5,
            }],
        ),
    ])
