"""
06_include.launch.py
ROS2 Launch IncludeLaunchDescription 示例

知识点：
- IncludeLaunchDescription：包含其他 launch 文件
- PythonLaunchDescriptionSource：包含 Python 格式的 launch
- 传递参数给子 launch 文件
- launch 文件的模块化组织
- 两种 include 方式：包内引用 vs 路径引用

运行方式：
    ros2 launch launch_learning 06_include.launch.py
    ros2 launch launch_learning 06_include.launch.py num_robots:=2
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # 1. IncludeLaunchDescription：包含其他 launch 文件
    #    将另一个 launch 文件的内容嵌入到当前 launch 中
    #    好处：
    #    - 模块化组织 launch 文件
    #    - 复用已有 launch 文件
    #    - 支持传递参数
    # ============================================================

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='启动机器人数量（1或2）'
    )

    # ============================================================
    # 2. 方式一：通过包名引用子 launch 文件（推荐）
    #    使用 FindPackageShare 定位包的 share 目录
    #    这样即使包安装路径变化，也能正确找到 launch 文件
    # ============================================================

    # 获取当前包的 share 目录
    pkg_share = FindPackageShare('launch_learning').find('launch_learning')

    # 包含 01_basic.launch.py
    basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', '01_basic.launch.py')
        ),
        # launch_arguments：传递参数给子 launch 文件
        # 格式：dict，键为子 launch 中的 DeclareLaunchArgument 名
        # 注意：01_basic.launch.py 没有声明参数，这里仅为演示
        # launch_arguments={
        #     'some_arg': 'some_value',
        # },
    )

    # ============================================================
    # 3. 方式二：包含子目录中的 launch 文件
    #    子 launch 文件放在 sub_launch/ 目录下
    # ============================================================
    sub_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sub_launch', 'robot_bringup.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'use_sim': 'false',
        },
    )

    # ============================================================
    # 4. 包含其他包的 launch 文件
    #    可以跨包引用 launch 文件
    # ============================================================
    # 示例：包含 tf2_ros 包的示例 launch（如果存在）
    # tf2_pkg_share = FindPackageShare('tf2_ros').find('tf2_ros')
    # tf2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(tf2_pkg_share, 'launch', 'some_file.launch.py')
    #     ),
    # )

    # ============================================================
    # 5. 多机器人场景：用 GroupAction + IncludeLaunchDescription
    #    为每个机器人的子 launch 设置不同的命名空间
    # ============================================================
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sub_launch', 'robot_bringup.launch.py')
            ),
            launch_arguments={
                'robot_name': 'robot2',
                'use_sim': 'false',
            },
        ),
    ])

    return LaunchDescription([
        num_robots_arg,
        # 包含基础示例
        # basic_launch,       # 取消注释可同时启动基础示例中的节点

        # 包含子 launch
        sub_launch_file,

        # 第二个机器人（取消注释启用）
        # robot2_group,
    ])
