"""多容器启动 —— 组件分布在多个容器中，部分进程内通信、部分跨进程通信"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        # 容器 1：数据采集（source + filter 同进程，进程内通信）
        ComposableNodeContainer(
            name='acquisition_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_learning_cpp',
                    plugin='SensorSourceComponent',
                    name='sensor_source',
                    parameters=[{
                        'publish_rate': 5.0,
                        'base_temperature': 25.0,
                        'noise_amplitude': 2.0,
                    }],
                ),
                ComposableNode(
                    package='component_learning_cpp',
                    plugin='DataFilterComponent',
                    name='data_filter',
                    parameters=[{
                        'window_size': 5,
                    }],
                ),
            ],
            output='screen',
        ),
        # 容器 2：数据显示（独立进程，通过 DDS 与容器 1 通信）
        ComposableNodeContainer(
            name='display_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_learning_cpp',
                    plugin='DataDisplayComponent',
                    name='data_display',
                    parameters=[{
                        'input_topic': 'filtered_temperature',
                    }],
                ),
            ],
            output='screen',
        ),
    ])
