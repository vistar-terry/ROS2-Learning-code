"""
⑪ EventsCBGExecutor —— 事件驱动 + 回调组（CBG）执行器
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_learning_cpp',
            executable='events_cbg_executor',
            name='events_cbg_executor',
            output='screen',
            emulate_tty=True,
        ),
    ])
