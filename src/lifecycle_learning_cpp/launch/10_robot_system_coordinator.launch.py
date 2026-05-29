"""⑩ 机器人多子系统协调 —— 多生命周期节点的有序启停

启动4个生命周期节点 + 1个协调器
协调器自动按依赖顺序启停所有子系统
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    # 4 个生命周期子系统节点（使用简化的生命周期节点模拟）
    imu_driver = LifecycleNode(
        package='lifecycle_learning_cpp',
        executable='robot_sensor_driver',
        name='imu_driver',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    nav_controller = LifecycleNode(
        package='lifecycle_learning_cpp',
        executable='robot_nav_lifecycle',
        name='controller',
        namespace='',  # 协调器查找的是 /controller
        output='screen',
        emulate_tty=True,
    )

    # 协调器节点
    coordinator = Node(
        package='lifecycle_learning_cpp',
        executable='robot_system_coordinator',
        name='system_coordinator',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        imu_driver,
        nav_controller,
        coordinator,
    ])
