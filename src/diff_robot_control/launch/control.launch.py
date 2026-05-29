"""
launch/control.launch.py
Gazebo Harmonic + ros2_control 控制器启动 launch 文件

功能：
- 启动 Gazebo Harmonic 仿真
- 加载差速控制器和关节状态广播器
- 可选启动 RViz 和 teleop

运行方式：
    ros2 launch diff_robot_control control.launch.py
    ros2 launch diff_robot_control control.launch.py use_rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('diff_robot_description').find('diff_robot_description')
    control_pkg = FindPackageShare('diff_robot_control').find('diff_robot_control')

    default_model_path = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'diff_robot.rviz')
    controllers_config = os.path.join(control_pkg, 'config', 'diff_bot_controllers.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    # 声明参数
    model_arg = DeclareLaunchArgument('model', default_value=default_model_path)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # 处理 URDF
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])

    # ============================================================
    # 1. 启动 Gazebo Harmonic (使用 gz sim 命令)
    # ============================================================
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', 'empty.sdf'],
        output='screen',
    )

    # ============================================================
    # 2. robot_state_publisher
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    # ============================================================
    # 3. 生成机器人到 Gazebo (使用 gz service call)
    # ============================================================
    spawn_entity = ExecuteProcess(
        cmd=['gz', 'service', 'call', '/world/empty/create', 
             'gz.msgs.EntityFactory', 
             '{sdf_filename: "$(find diff_robot_description)/urdf/diff_robot.urdf.xacro"}'],
        output='screen',
    )

    # ============================================================
    # 4. 启动 ros2_control 控制器
    #    通过 controller_manager 节点来管理控制器
    # ============================================================
    
    # 4a. Control Manager 节点
    # 负责加载和管理控制器
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content,
                    'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # 4b. 启动 joint_state_broadcaster
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    # 4c. 启动 diff_drive_controller
    diff_drive_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_bot_base_controller'],
        output='screen',
    )

    # ============================================================
    # 5. RViz
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        use_rviz_arg,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        control_node,
        # 在 spawn_entity 完成后启动控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    diff_drive_controller_spawner,
                ],
            )
        ),
        spawn_entity,
        rviz_node,
    ])
