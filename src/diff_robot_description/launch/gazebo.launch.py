"""
launch/gazebo.launch.py
在 Gazebo 中仿真机器人

功能：
- 启动 Gazebo 仿真环境
- 加载机器人模型到 Gazebo
- 启动 RViz2 可视化

运行方式：
    ros2 launch diff_robot_description gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('diff_robot_description').find('diff_robot_description')

    default_model_path = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'diff_robot.rviz')

    # 声明参数
    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='URDF 模型路径')

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_config,
        description='RViz 配置路径')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用仿真时间')

    # 处理 URDF
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])

    # ============================================================
    # 1. 启动 Gazebo 仿真器
    # ============================================================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'),
                        'launch', 'gzserver.launch.py')
        ),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'),
                        'launch', 'gzclient.launch.py')
        ),
    )

    # ============================================================
    # 2. robot_state_publisher
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ============================================================
    # 3. 在 Gazebo 中生成机器人模型
    # ============================================================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'diff_robot',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ============================================================
    # 4. RViz2 可视化
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        use_rviz_arg,
        use_sim_time_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
    ])
