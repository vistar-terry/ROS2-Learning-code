"""④ Action 目标策略：拒绝 vs 抢占

演示：
  Policy Server 支持 REJECT（忙碌时拒绝）和 PREEMPT（抢占旧目标）
  通过 policy 参数切换策略

用法：
  ros2 launch action_learning_cpp 04_action_goal_policy.launch.py
  ros2 launch action_learning_cpp 04_action_goal_policy.launch.py policy:=reject
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'policy', default_value='preempt',
            description='目标策略: preempt（抢占）或 reject（拒绝）'),
        # Policy Server
        Node(
            package='action_learning_cpp',
            executable='06_action_server_policy',
            name='policy_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'policy': LaunchConfiguration('policy'),
            }],
        ),
        # 简单 Client（复用基础客户端）
        Node(
            package='action_learning_cpp',
            executable='02_action_client_basic',
            name='policy_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target': 10,
            }],
        ),
    ])
