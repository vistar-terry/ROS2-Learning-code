"""
advanced/07_substitutions.launch.py
ROS2 Launch Substitutions（替换）高级示例

知识点：
- Substitution 的概念和类型
- TextSubstitution：纯文本
- LaunchConfiguration：运行时参数
- EnvironmentVariable：环境变量
- PathJoinSubstitution：路径拼接
- PythonExpression：Python 表达式
- FindExecutable / ExecuteProcess：执行命令
- Substitution 拼接和组合

运行方式：
    ros2 launch launch_learning 07_substitutions.launch.py
    ros2 launch launch_learning 07_substitutions.launch.py robot_name:=mybot
    ROS_DOMAIN_ID=42 ros2 launch launch_learning 07_substitutions.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # 1. Substitution 概念
    #    Substitution 是 launch 系统的核心概念
    #    它表示一个"在运行时才确定的值"
    #    所有 Substitution 在 launch 解析时被替换为实际值
    #
    #    支持列表拼接：
    #    ['prefix_', LaunchConfiguration('name'), '_suffix']
    #    等价于 "prefix_" + <name的值> + "_suffix"
    # ============================================================

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='机器人名称'
    )

    # ============================================================
    # 2. LaunchConfiguration：最常用的 Substitution
    #    获取 DeclareLaunchArgument 声明的参数值
    # ============================================================
    robot_name = LaunchConfiguration('robot_name')

    # ============================================================
    # 3. EnvironmentVariable：读取环境变量
    #    default_value 在环境变量不存在时使用
    # ============================================================
    domain_id = EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')

    # ============================================================
    # 4. PathJoinSubstitution：安全地拼接路径
    #    自动处理路径分隔符，比 os.path.join 更适合 launch 上下文
    # ============================================================
    config_path = PathJoinSubstitution([
        FindPackageShare('launch_learning'),   # 包的 share 目录
        'config',                               # 子目录
        'params.yaml',                          # 文件名
    ])

    # ============================================================
    # 5. PythonExpression：在 launch 中执行 Python 表达式
    #    可以进行数值计算、类型转换、逻辑判断等
    # ============================================================
    # 计算发布频率（双倍频率）
    double_freq = PythonExpression([
        'float("', LaunchConfiguration('freq', default='1.0'), '") * 2'
    ])

    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='1.0',
        description='基础发布频率（Hz），实际使用双倍值'
    )

    # ============================================================
    # 6. LogInfo 中使用 Substitution 拼接
    #    LogInfo 的 msg 参数接受字符串列表
    #    列表中的 Substitution 会被依次替换并拼接
    # ============================================================
    startup_log = LogInfo(msg=[
        '>>> 启动配置: robot_name=',
        robot_name,
        ', ROS_DOMAIN_ID=',
        domain_id,
        ', config_path=',
        config_path,
    ])

    # ============================================================
    # 7. 在节点中使用各种 Substitution
    # ============================================================
    talker_node = Node(
        package='launch_learning',
        executable='param_talker',
        name=['talker_', robot_name],           # 动态节点名
        output='screen',
        parameters=[{
            'topic_name': ['data_', robot_name], # 动态话题名
            'publish_freq': double_freq,          # Python表达式结果
            'message_prefix': ['[', robot_name, ']'],  # 拼接前缀
        }],
    )

    # ============================================================
    # 8. SetEnvironmentVariable：设置环境变量
    #    在 launch 启动前设置环境变量
    # ============================================================
    set_log_level = SetEnvironmentVariable(
        name='RCUTILS_CONSOLE_MIN_SEVERITY',
        value='INFO',      # 只显示 INFO 及以上级别的日志
    )

    return LaunchDescription([
        # 声明参数
        robot_name_arg,
        freq_arg,

        # 设置环境变量
        set_log_level,

        # 启动日志（使用 Substitution 拼接）
        startup_log,

        # 启动节点
        talker_node,
    ])
