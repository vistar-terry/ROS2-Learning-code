"""⑥ 机械臂控制实战

演示：
  自定义 MoveArm Action 消息
  机械臂关节运动模拟
  速度因子控制
  3步抓取任务序列

自包含：Server + Client 在同一进程

用法：
  ros2 launch action_learning_cpp 06_robot_arm.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_learning_cpp',
            executable='08_robot_arm_demo',
            name='robot_arm_demo',
            output='screen',
            emulate_tty=True,
        ),
    ])
