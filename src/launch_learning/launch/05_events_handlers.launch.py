"""
05_events_handlers.launch.py
ROS2 Launch 事件与处理器示例

知识点：
- RegisterEventHandler：注册事件处理器
- OnProcessStart：节点启动事件
- OnProcessExit：节点退出事件
- OnExecutionComplete：动作执行完成事件
- EmitEvent：发射自定义事件
- OpaqueFunction：在 launch 中执行任意 Python 函数
- 节点退出后重启或执行其他动作

运行方式：
    ros2 launch launch_learning 05_events_handlers.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # 1. OnProcessStart：节点启动时触发
    #    常用于：等待某个节点就绪后再执行操作
    # ============================================================
    worker_node = Node(
        package='launch_learning',
        executable='worker_node',
        name='worker',
        output='screen',
        parameters=[{
            'max_count': 5,         # 5步后退出
            'work_interval': 1.0,
        }],
    )

    # 当 worker 启动时，打印日志
    on_worker_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=worker_node,    # 监听的目标动作
            on_start=[
                LogInfo(msg='>>> 工作节点已启动！开始处理任务...'),
            ],
        )
    )

    # ============================================================
    # 2. OnProcessExit：节点退出时触发
    #    常用于：
    #    - 节点退出后启动另一个节点
    #    - 节点退出后执行清理操作
    #    - 节点退出后关闭整个 launch
    # ============================================================

    # 当 worker 退出后，启动总结节点
    summary_node = Node(
        package='launch_learning',
        executable='param_talker',
        name='summary',
        output='screen',
        parameters=[{
            'topic_name': 'summary',
            'message_prefix': '[任务完成]',
            'publish_freq': 1.0,
        }],
    )

    on_worker_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=worker_node,
            on_exit=[
                LogInfo(msg='>>> 工作节点已退出！启动总结节点...'),
                # 节点退出后启动新节点
                # 注意：这里直接把 summary_node 放在 on_exit 列表中
                summary_node,
            ],
        )
    )

    # ============================================================
    # 3. 延迟启动节点
    #    使用 TimerAction 在指定时间后启动节点
    # ============================================================
    delayed_listener = Node(
        package='launch_learning',
        executable='ns_listener',
        name='delayed_listener',
        output='screen',
    )

    # 3秒后启动监听器
    delayed_start = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='>>> 3秒延迟到达，启动监听器...'),
            delayed_listener,
        ],
    )

    # ============================================================
    # 4. OpaqueFunction：在 launch 中执行任意 Python 代码
    #    用于在 launch 解析阶段执行复杂逻辑
    #    OpaqueFunction 接收一个 context 参数，可以访问 launch 上下文
    # ============================================================
    def log_launch_info(context):
        """在 launch 启动时打印信息"""
        # context.launch_configurations 可以获取所有 LaunchConfiguration 的值
        mode = context.launch_configurations.get('mode', 'run')
        return [LogInfo(msg=f'>>> Launch 模式: {mode}')]

    log_info_action = OpaqueFunction(function=log_launch_info)

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='run',
        description='运行模式'
    )

    # ============================================================
    # 5. 多个节点退出后关闭整个 launch
    #    使用 EmitEvent 发射 Shutdown 事件
    # ============================================================
    # 短暂运行的工作节点2
    worker2_node = Node(
        package='launch_learning',
        executable='worker_node',
        name='worker2',
        output='screen',
        parameters=[{
            'max_count': 3,
            'work_interval': 0.5,
        }],
    )

    on_worker2_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=worker2_node,
            on_exit=[
                LogInfo(msg='>>> worker2 已退出，关闭所有节点...'),
                EmitEvent(event=Shutdown(reason='所有工作已完成')),
            ],
        )
    )

    return LaunchDescription([
        # 声明参数
        mode_arg,

        # OpaqueFunction
        log_info_action,

        # 主工作节点
        worker_node,
        on_worker_start,
        on_worker_exit,

        # 延迟启动
        delayed_start,

        # 工作节点2 + 退出后关闭
        worker2_node,
        on_worker2_exit,
    ])
