"""
04_conditions.launch.py
ROS2 Launch 条件启动示例

知识点：
- IfCondition：条件为 True 时执行
- UnlessCondition：条件为 False 时执行
- LaunchConfiguration 配合条件判断
- PythonExpression：复杂条件表达式
- 环境变量条件

运行方式：
    ros2 launch launch_learning 04_conditions.launch.py
    ros2 launch launch_learning 04_conditions.launch.py use_listener:=true
    ros2 launch launch_learning 04_conditions.launch.py mode:=debug
    ros2 launch launch_learning 04_conditions.launch.py use_sim:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # 1. 简单布尔条件参数
    # ============================================================
    use_listener_arg = DeclareLaunchArgument(
        'use_listener',
        default_value='true',       # 字符串 'true' / 'false'
        description='是否启动监听器节点（true/false）'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='是否使用仿真模式（true/false）'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='run',
        description='运行模式：run | debug'
    )

    # ============================================================
    # 2. IfCondition：条件为 True 时执行
    #    LaunchConfiguration('use_listener') 返回字符串 'true'
    #    IfCondition 会将 'true' 转为 True，'false' 转为 False
    # ============================================================
    talker_node = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker',
        output='screen',
        # 无论如何都启动
    )

    listener_node = Node(
        package='launch_learning',
        executable='ns_listener',
        name='listener',
        output='screen',
        # --- 仅当 use_listener=true 时启动 ---
        condition=IfCondition(LaunchConfiguration('use_listener')),
    )

    # ============================================================
    # 3. UnlessCondition：条件为 False 时执行
    #    与 IfCondition 相反
    # ============================================================
    real_sensor_node = Node(
        package='launch_learning',
        executable='worker_node',
        name='real_sensor',
        output='screen',
        parameters=[{
            'max_count': 0,
            'work_interval': 2.0,
        }],
        # --- 除非 use_sim=true，否则启动（即真实模式下启动） ---
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
    )

    # ============================================================
    # 4. PythonExpression：复杂条件表达式
    #    可以在 launch 中写 Python 表达式进行条件判断
    #    支持：比较运算、逻辑运算、字符串操作等
    # ============================================================
    debug_talker_node = Node(
        package='launch_learning',
        executable='param_talker',
        name='debug_talker',
        output='screen',
        parameters=[{
            'topic_name': 'debug_chatter',
            'message_prefix': '[DEBUG]',
            'publish_freq': 5.0,
        }],
        # --- 当 mode == 'debug' 时启动 ---
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('mode'), '" == "debug"'
            ])
        ),
    )

    # 复合条件：use_sim=true AND use_listener=true
    sim_listener_node = Node(
        package='launch_learning',
        executable='ns_listener',
        name='sim_listener',
        output='screen',
        parameters=[{
            'topic_name': 'sim_data',
        }],
        # --- 仿真模式下且启用监听器时启动 ---
        condition=IfCondition(
            PythonExpression([
                LaunchConfiguration('use_sim'), ' and ',
                LaunchConfiguration('use_listener'),
            ])
        ),
    )

    # ============================================================
    # 5. 环境变量条件
    #    可以根据环境变量决定是否启动节点
    # ============================================================
    env_sim_node = Node(
        package='launch_learning',
        executable='worker_node',
        name='env_sim_worker',
        output='screen',
        parameters=[{
            'max_count': 5,
        }],
        # --- 当环境变量 ROS_DOMAIN_ID 存在时启动 ---
        condition=IfCondition(
            PythonExpression([
                '"', EnvironmentVariable('ROS_DOMAIN_ID', default_value=''), '" != ""'
            ])
        ),
    )

    return LaunchDescription([
        # 声明参数
        use_listener_arg,
        use_sim_arg,
        mode_arg,

        # 节点
        talker_node,
        listener_node,
        real_sensor_node,
        debug_talker_node,
        sim_listener_node,
        env_sim_node,
    ])
