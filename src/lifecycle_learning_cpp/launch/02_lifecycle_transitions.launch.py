"""② 状态转换详解 —— 包含失败和错误恢复演示"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='lifecycle_transitions',
            name='lifecycle_transitions',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
    ])
