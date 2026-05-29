"""⑦ 生命周期回调组 —— LifecycleNode + CallbackGroup + MultiThreadedExecutor"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='lifecycle_callback_groups',
            name='lifecycle_cb_groups',
            namespace='',
            output='screen',
            emulate_tty=True,
        ),
    ])
