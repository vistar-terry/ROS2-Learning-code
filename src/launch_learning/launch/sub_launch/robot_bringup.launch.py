"""
sub_launch/robot_bringup.launch.py
机器人启动子 launch 文件 — 供 06_include.launch.py 调用

功能：启动单个机器人的所有节点
演示：子 launch 文件如何接收和传递参数

运行方式：
    # 通常由父 launch 文件 IncludeLaunchDescription 调用
    # 也可单独运行：
    ros2 launch launch_learning robot_bringup.launch.py robot_name:=robot1 use_sim:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # 1. 子 launch 文件声明自己的参数
    #    父 launch 文件通过 launch_arguments 传入
    #    也可独立运行时从命令行传入
    # ============================================================
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='机器人名称（用作命名空间）'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='是否使用仿真模式'
    )

    # 获取参数值
    robot_name = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')

    # ============================================================
    # 2. 启动机器人节点
    #    每个机器人都需要发布者和监听者
    # ============================================================
    talker = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker',
        namespace=robot_name,        # 使用参数作为命名空间
        output='screen',
        parameters=[{
            'topic_name': 'chatter',
            'message_prefix': ['[', robot_name, ']'],  # Substitution 拼接
            'publish_freq': 1.0,
        }],
    )

    listener = Node(
        package='launch_learning',
        executable='ns_listener',
        name='listener',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'topic_name': 'chatter',
        }],
    )

    return LaunchDescription([
        # 声明参数
        robot_name_arg,
        use_sim_arg,

        # 启动日志
        LogInfo(msg=['>>> 启动机器人: ', robot_name]),

        # 启动节点
        talker,
        listener,
    ])
