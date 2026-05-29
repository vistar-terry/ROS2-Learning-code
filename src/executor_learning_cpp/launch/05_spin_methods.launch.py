"""
⑤ spin / spin_once / spin_some —— 三种驱动方式

用法：
  ros2 launch executor_learning_cpp 05_spin_methods.launch.py mode:=1  # spin()
  ros2 launch executor_learning_cpp 05_spin_methods.launch.py mode:=2  # spin_once()
  ros2 launch executor_learning_cpp 05_spin_methods.launch.py mode:=3  # spin_some()
  ros2 launch executor_learning_cpp 05_spin_methods.launch.py mode:=4  # 自定义循环
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='1',
        description='驱动模式: 1=spin(), 2=spin_once(), 3=spin_some(), 4=自定义循环'
    )

    spin_node = Node(
        package='executor_learning_cpp',
        executable='spin_methods',
        name='spin_methods',
        output='screen',
        emulate_tty=True,
        arguments=[LaunchConfiguration('mode')],
    )

    return LaunchDescription([mode_arg, spin_node])
