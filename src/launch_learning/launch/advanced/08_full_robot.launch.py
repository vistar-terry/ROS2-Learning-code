"""
advanced/08_full_robot.launch.py
ROS2 Launch 综合实战 — 多机器人导航启动文件

知识点：
- 多机器人系统的 launch 组织方式
- 参数文件 + 命令行参数混合使用
- 条件启动 + 事件处理综合运用
- GroupAction 命名空间隔离
- 完整的 launch 文件设计模式

场景：模拟一个多机器人导航系统
- robot1: 实际机器人（使用真实传感器）
- robot2: 仿真机器人（使用仿真传感器）
- 可选的 RViz 可视化
- 节点退出后的自动重启

运行方式：
    ros2 launch launch_learning 08_full_robot.launch.py
    ros2 launch launch_learning 08_full_robot.launch.py num_robots:=2 use_rviz:=true
    ros2 launch launch_learning 08_full_robot.launch.py use_sim:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # 1. 全局参数声明
    # ============================================================
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='启动机器人数量（1 或 2）'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='是否使用仿真模式'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动 RViz2 可视化'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='单机器人模式下的机器人名称'
    )

    # 获取参数
    num_robots = LaunchConfiguration('num_robots')
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')

    # 包路径
    pkg_share = FindPackageShare('launch_learning').find('launch_learning')

    # ============================================================
    # 2. Robot1 组 — 实际机器人
    #    GroupAction + PushRosNamespace 实现命名空间隔离
    # ============================================================
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        # 包含子 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sub_launch', 'robot_bringup.launch.py')
            ),
            launch_arguments={
                'robot_name': 'robot1',
                'use_sim': 'false',
            },
        ),
    ])

    # ============================================================
    # 3. Robot2 组 — 仅当 num_robots >= 2 时启动
    # ============================================================
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sub_launch', 'robot_bringup.launch.py')
            ),
            launch_arguments={
                'robot_name': 'robot2',
                'use_sim': use_sim,
            },
        ),
    ])

    # 条件：仅当 num_robots >= 2 时启动 robot2
    robot2_conditional = GroupAction(
        actions=[robot2_group],
        condition=IfCondition(
            PythonExpression([num_robots, ' >= 2'])
        ),
    )

    # ============================================================
    # 4. 仿真模式节点 — 仅在仿真模式下启动
    # ============================================================
    sim_bridge = Node(
        package='launch_learning',
        executable='worker_node',
        name='sim_bridge',
        namespace='simulation',
        output='screen',
        parameters=[{
            'max_count': 0,
            'work_interval': 0.5,
        }],
        condition=IfCondition(use_sim),
    )

    # ============================================================
    # 5. RViz2 可视化节点
    # ============================================================
    rviz_config = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # ============================================================
    # 6. 全局参数服务器（可选）
    #    演示在 launch 中直接设置 ROS2 参数
    # ============================================================
    global_param_node = Node(
        package='launch_learning',
        executable='worker_node',
        name='global_monitor',
        output='screen',
        parameters=[{
            'max_count': 0,
            'work_interval': 5.0,  # 每5秒报告一次
        }],
        condition=UnlessCondition(use_sim),  # 非仿真模式才启动
    )

    # ============================================================
    # 7. 设置环境变量
    # ============================================================
    set_log_format = SetEnvironmentVariable(
        name='RCUTILS_COLORIZED_OUTPUT',
        value='1',
    )

    # ============================================================
    # 8. 启动日志
    # ============================================================
    startup_log = LogInfo(msg=[
        '\n========================================\n',
        '  多机器人导航系统启动\n',
        '  机器人数量: ', num_robots, '\n',
        '  仿真模式: ', use_sim, '\n',
        '  RViz可视化: ', use_rviz, '\n',
        '========================================',
    ])

    return LaunchDescription([
        # 参数声明
        num_robots_arg,
        use_sim_arg,
        use_rviz_arg,
        robot_name_arg,

        # 环境变量
        set_log_format,

        # 启动日志
        startup_log,

        # 机器人1（始终启动）
        robot1_group,

        # 机器人2（条件启动）
        robot2_conditional,

        # 仿真桥接（条件启动）
        sim_bridge,

        # 全局监控（条件启动）
        global_param_node,

        # RViz（条件启动）
        rviz_node,
    ])
