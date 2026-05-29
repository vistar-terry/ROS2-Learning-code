"""① 基础生命周期节点 —— 手动状态切换演示"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='lifecycle_basic',
            name='lifecycle_basic',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
    ])
