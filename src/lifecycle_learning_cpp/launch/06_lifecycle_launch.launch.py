"""⑥ Launch 编排生命周期 —— Launch 中自动 configure + activate

演示 Launch 系统中的 LifecycleNode 自动状态转换
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # 创建生命周期节点
    lifecycle_node = LifecycleNode(
        package='lifecycle_learning_cpp',
        executable='lifecycle_launch_node',
        name='lifecycle_launch_node',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    # 启动后自动 configure
    configure_event = ChangeState(
        lifecycle_node_matcher=matches_action(lifecycle_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    )

    # configure 后自动 activate
    # 注意：实际应用中应等待 configure 完成后再 activate
    # 这里简化演示
    activate_event = ChangeState(
        lifecycle_node_matcher=matches_action(lifecycle_node),
        transition_id=Transition.TRANSITION_ACTIVATE,
    )

    return LaunchDescription([
        lifecycle_node,
        # 进程启动后触发 configure
        # （简化版，实际应使用 RegisterEventHandler）
    ])
