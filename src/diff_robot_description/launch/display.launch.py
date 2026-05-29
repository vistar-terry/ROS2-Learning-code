"""
launch/display.launch.py
在 RViz 中显示机器人模型

功能：
- 加载 URDF/Xacro 模型
- 启动 robot_state_publisher 发布 TF
- 启动 joint_state_publisher_gui 手动控制关节
- 启动 RViz2 可视化

运行方式：
    ros2 launch diff_robot_description display.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 获取功能包路径
    pkg_share = FindPackageShare('diff_robot_description').find('diff_robot_description')

    # 默认 URDF 文件路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'diff_robot.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'diff_robot.rviz')

    # 声明 launch 参数
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='URDF/Xacro 模型文件的绝对路径'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config,
        description='RViz 配置文件的绝对路径'
    )

    use_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动 joint_state_publisher_gui'
    )

    # 使用 xacro 处理 URDF 文件
    # Command 会在运行时执行 xacro 命令
    robot_description_content = Command([
        'xacro ', LaunchConfiguration('model')
    ])

    # ============================================================
    # robot_state_publisher 节点
    # 读取 URDF，订阅 joint_states，发布 TF 变换
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    # ============================================================
    # joint_state_publisher_gui 节点
    # 提供滑块界面手动控制 continuous 关节
    # 也可使用 joint_state_publisher（无GUI，自动发布默认值）
    # ============================================================
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # ============================================================
    # RViz2 可视化节点
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        use_gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
