"""④ 生命周期订阅器 —— 按状态控制订阅行为"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='lifecycle_subscriber',
            name='lifecycle_subscriber',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
    ])
