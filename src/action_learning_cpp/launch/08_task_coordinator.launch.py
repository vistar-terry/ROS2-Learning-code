"""⑧ 多 Action 协调器

演示：
  同时管理机械臂和导航两个 Action Server
  并行 + 顺序混合任务编排
  "导航到货架 → 抓取物品 → 返回起点" 复合工作流

需要同时启动 arm_server 和 nav_server：
  协调器 (10_task_coordinator) + arm_server (08_robot_arm_demo 的 server 部分)
  + nav_server (09_robot_nav_demo 的 server 部分)

用法：
  ros2 launch action_learning_cpp 08_task_coordinator.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 机械臂 Action Server
        Node(
            package='action_learning_cpp',
            executable='08_robot_arm_demo',
            name='robot_arm_server',
            output='screen',
            emulate_tty=True,
            # 只需要 server，client 的目标会因为 server 已存在而一起发送
            # 实际部署中应拆分为独立 server 可执行文件
        ),
        # 导航 Action Server
        Node(
            package='action_learning_cpp',
            executable='09_robot_nav_demo',
            name='robot_nav_server',
            output='screen',
            emulate_tty=True,
        ),
        # 任务协调器
        Node(
            package='action_learning_cpp',
            executable='10_task_coordinator',
            name='task_coordinator',
            output='screen',
            emulate_tty=True,
        ),
    ])
