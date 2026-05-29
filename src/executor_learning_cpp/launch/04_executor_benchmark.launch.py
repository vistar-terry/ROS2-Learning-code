"""
④ 执行器对比基准 —— 三种执行器性能测试

用法：
  ros2 launch executor_learning_cpp 04_executor_benchmark.launch.py mode:=1  # SingleThreaded
  ros2 launch executor_learning_cpp 04_executor_benchmark.launch.py mode:=2  # MultiThreaded
  ros2 launch executor_learning_cpp 04_executor_benchmark.launch.py mode:=3  # StaticSingleThreaded
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='1',
        description='执行器模式: 1=SingleThreaded, 2=MultiThreaded, 3=StaticSingleThreaded'
    )

    benchmark_node = Node(
        package='executor_learning_cpp',
        executable='executor_benchmark',
        name='executor_benchmark',
        output='screen',
        emulate_tty=True,
        arguments=[LaunchConfiguration('mode')],
    )

    return LaunchDescription([mode_arg, benchmark_node])
