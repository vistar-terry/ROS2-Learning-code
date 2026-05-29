"""⑨ 机器人导航子系统 —— 生命周期控制的安全关键系统"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='robot_nav_lifecycle',
            name='nav_controller',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'control_rate': 50,
                'planning_rate': 0.5,
                'max_speed': 1.0,
                'safety_distance': 0.3,
            }],
        ),
    ])
