"""⑦ 机器人导航实战

演示：
  自定义 Navigate Action 消息
  2D 平面导航模拟
  多路径点巡逻任务
  障碍物检测与避障（模拟）

自包含：Server + Client 在同一进程

用法：
  ros2 launch action_learning_cpp 07_robot_nav.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_learning_cpp',
            executable='09_robot_nav_demo',
            name='robot_nav_demo',
            output='screen',
            emulate_tty=True,
        ),
    ])
