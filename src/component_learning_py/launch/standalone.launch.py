"""Python 组件启动 —— 独立进程方式运行（每个组件独立进程）"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """以独立节点方式启动 Python 组件"""
    return LaunchDescription([
        Node(
            package='component_learning_py',
            executable='sensor_source',
            name='sensor_source',
            parameters=[{
                'publish_rate': 2.0,
                'base_temperature': 25.0,
                'noise_amplitude': 3.0,
            }],
            output='screen',
        ),
        Node(
            package='component_learning_py',
            executable='data_filter',
            name='data_filter',
            parameters=[{
                'window_size': 5,
            }],
            output='screen',
        ),
        Node(
            package='component_learning_py',
            executable='data_display',
            name='data_display',
            output='screen',
        ),
    ])
