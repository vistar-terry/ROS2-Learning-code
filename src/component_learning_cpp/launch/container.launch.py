"""单容器启动 —— 所有组件运行在同一进程中，启用进程内零拷贝通信"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='sensor_pipeline_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            # ★ 关键：将所有组件放在同一个容器中
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
                        'input_topic': 'raw_temperature',
                        'output_topic': 'filtered_temperature',
                    }],
                ),
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
