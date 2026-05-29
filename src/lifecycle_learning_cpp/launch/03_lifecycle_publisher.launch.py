"""③ 生命周期发布器 —— LifecyclePublisher 按状态自动启用/禁用"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='lifecycle_publisher',
            name='lifecycle_publisher',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
    ])
