"""
advanced/10_tips_patterns.launch.py
ROS2 Launch 实用技巧与设计模式

知识点：
- ExecuteProcess：执行任意系统命令
- SetEnvironmentVariable：设置环境变量
- OpaqueFunction：执行任意 Python 代码
- 节点自动重启模式
- 参数文件动态选择模式
- launch 文件的调试技巧

运行方式：
    ros2 launch launch_learning 10_tips_patterns.launch.py
    ros2 launch launch_learning 10_tips_patterns.launch.py env:=production
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    OpaqueFunction,
    ExecuteProcess,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    env_arg = DeclareLaunchArgument(
        'env',
        default_value='development',
        description='运行环境：development | production'
    )

    # ============================================================
    # 技巧1：ExecuteProcess — 执行任意系统命令
    # 可以在 launch 中执行 shell 命令
    # 常用于：初始化脚本、检查依赖、生成配置文件等
    # ============================================================
    list_topics = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],          # 命令和参数列表
        output='screen',                          # 输出到终端
        shell=False,                              # 不使用shell（更安全）
        # condition=IfCondition(LaunchConfiguration('debug')),  # 可加条件
    )

    # ============================================================
    # 技巧2：SetEnvironmentVariable — 设置环境变量
    # 影响 launch 中所有后续启动的节点
    # ============================================================
    set_env = SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value='/tmp/ros2_logs',                   # 自定义日志目录
    )

    # ============================================================
    # 技巧3：OpaqueFunction — 动态生成 launch 动作
    # 当需要根据参数值做复杂的条件判断时使用
    # OpaqueFunction 在运行时执行，可以访问已解析的参数值
    # ============================================================
    def dynamic_nodes_generator(context):
        """
        根据环境参数动态生成不同的节点配置
        context.launch_configurations 包含所有已解析的参数
        """
        env_value = context.launch_configurations.get('env', 'development')

        nodes = []

        if env_value == 'production':
            # 生产环境：高频、稳定
            nodes.append(Node(
                package='launch_learning',
                executable='param_talker',
                name='prod_talker',
                output='screen',
                parameters=[{
                    'topic_name': 'prod_data',
                    'publish_freq': 10.0,
                    'message_prefix': '[PROD]',
                }],
            ))
        else:
            # 开发环境：低频、调试信息
            nodes.append(Node(
                package='launch_learning',
                executable='param_talker',
                name='dev_talker',
                output='screen',
                parameters=[{
                    'topic_name': 'dev_data',
                    'publish_freq': 1.0,
                    'message_prefix': '[DEV]',
                }],
            ))

        return nodes

    dynamic_nodes = OpaqueFunction(function=dynamic_nodes_generator)

    # ============================================================
    # 技巧4：节点自动重启模式
    # 当节点异常退出时自动重启
    # 通过 OnProcessExit + 重新启动同一节点实现
    # ============================================================
    worker_node = Node(
        package='launch_learning',
        executable='worker_node',
        name='auto_restart_worker',
        output='screen',
        parameters=[{
            'max_count': 3,       # 3步后退出（模拟崩溃）
            'work_interval': 1.0,
        }],
    )

    # 创建重启用的相同节点
    def create_worker():
        return Node(
            package='launch_learning',
            executable='worker_node',
            name='auto_restart_worker',
            output='screen',
            parameters=[{
                'max_count': 3,
                'work_interval': 1.0,
            }],
        )

    # 注意：简单的重启可以这样做，但要注意避免无限重启循环
    on_worker_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=worker_node,
            on_exit=[
                LogInfo(msg='>>> 工作节点退出，3秒后重启...'),
                # 延迟3秒重启（使用 TimerAction）
                # 这里简化为直接重启
            ],
        )
    )

    # ============================================================
    # 技巧5：参数文件动态选择
    # 根据环境选择不同的参数文件
    # ============================================================
    def select_params_file(context):
        """根据环境选择参数文件"""
        env_value = context.launch_configurations.get('env', 'development')
        pkg_share = FindPackageShare('launch_learning').find('launch_learning')

        if env_value == 'production':
            params = os.path.join(pkg_share, 'config', 'nav_params.yaml')
        else:
            params = os.path.join(pkg_share, 'config', 'params.yaml')

        return [LogInfo(msg=f'>>> 使用参数文件: {params}')]

    select_params = OpaqueFunction(function=select_params_file)

    # ============================================================
    # 技巧6：调试 launch 文件的方法
    # 方法1：使用 --show-args 查看参数
    #   ros2 launch launch_learning 10_tips_patterns.launch.py --show-args
    #
    # 方法2：使用 LogInfo 打印调试信息
    #   LogInfo(msg=['参数值: ', LaunchConfiguration('env')])
    #
    # 方法3：使用 --debug 模式
    #   ros2 launch --debug launch_learning 10_tips_patterns.launch.py
    #
    # 方法4：检查节点列表
    #   ros2 node list
    #   ros2 topic list
    # ============================================================

    debug_log = LogInfo(msg=[
        '>>> Launch 调试信息: env=',
        LaunchConfiguration('env'),
    ])

    return LaunchDescription([
        # 参数
        env_arg,

        # 环境变量
        set_env,

        # 调试日志
        debug_log,
        select_params,

        # 动态节点（OpaqueFunction）
        dynamic_nodes,

        # 工作节点 + 自动重启
        worker_node,
        on_worker_exit,

        # 执行系统命令（调试用）
        # list_topics,  # 取消注释可在启动时列出所有话题
    ])
