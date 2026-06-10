"""独立进程启动 —— 每个组件运行在独立进程中（等同传统节点方式）"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        # 每个组件在独立的单组件容器中运行（等同独立进程）
        ComposableNodeContainer(
            name='sensor_source_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_learning_cpp',
                    plugin='SensorSourceComponent',
                    name='sensor_source',
                    parameters=[{
                        'publish_rate': 2.0,
                        'base_temperature': 25.0,
                        'noise_amplitude': 3.0,
                    }],
                ),
            ],
            output='screen',
        ),
        ComposableNodeContainer(
            name='data_filter_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
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
        ComposableNodeContainer(
            name='data_display_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_learning_cpp',
                    plugin='DataDisplayComponent',
                    name='data_display',
                ),
            ],
            output='screen',
        ),
    ])
