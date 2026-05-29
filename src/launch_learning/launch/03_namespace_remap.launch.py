"""
03_namespace_remap.launch.py
ROS2 Launch 命名空间与重映射示例

知识点：
- namespace：节点和话题的命名空间隔离
- remappings：话题/服务重映射
- 多实例启动：相同节点启动多个副本
- group actions：命名空间分组

运行方式：
    ros2 launch launch_learning 03_namespace_remap.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # ============================================================
    # 1. 命名空间（namespace）
    #    设置后，节点名变为 /namespace/node_name
    #    话题名变为 /namespace/topic_name
    #    用途：多机器人系统中隔离不同机器人的话题
    # ============================================================
    talker_ns_robot1 = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker',
        namespace='robot1',          # 设置命名空间为 /robot1
        output='screen',
        parameters=[{
            'topic_name': 'chatter',
            'message_prefix': '[robot1]',
        }],
    )

    talker_ns_robot2 = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker',
        namespace='robot2',          # 设置命名空间为 /robot2
        output='screen',
        parameters=[{
            'topic_name': 'chatter',
            'message_prefix': '[robot2]',
        }],
    )

    # ============================================================
    # 2. 话题重映射（remappings）
    #    将节点内部的话题名映射到外部不同的话题名
    #    格式：[(原始话题名, 映射后话题名)]
    #    用途：不改代码的情况下适配不同的话题名
    # ============================================================
    talker_remapped = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker_remapped',
        output='screen',
        parameters=[{
            'topic_name': 'chatter',         # 节点内部话题名
            'message_prefix': '[remapped]',
        }],
        # 将 /chatter 重映射为 /sensor_data
        remappings=[
            ('chatter', 'sensor_data'),
        ],
    )

    listener_remapped = Node(
        package='launch_learning',
        executable='ns_listener',
        name='listener_remapped',
        output='screen',
        parameters=[{
            'topic_name': 'sensor_data',     # 监听重映射后的话题
        }],
    )

    # ============================================================
    # 3. GroupAction + PushRosNamespace
    #    为一组动作统一设置命名空间
    #    比 Node 的 namespace 参数更灵活，适用于一组节点共享命名空间
    # ============================================================
    robot3_group = GroupAction([
        # 将组内所有节点推入 /robot3 命名空间
        PushRosNamespace('robot3'),

        # 组内的节点自动获得 /robot3 命名空间
        Node(
            package='launch_learning',
            executable='param_talker',
            name='talker',
            output='screen',
            parameters=[{
                'topic_name': 'chatter',
                'message_prefix': '[robot3]',
            }],
        ),
        Node(
            package='launch_learning',
            executable='ns_listener',
            name='listener',
            output='screen',
            parameters=[{
                'topic_name': 'chatter',
            }],
        ),
    ])

    return LaunchDescription([
        # --- 命名空间示例 ---
        talker_ns_robot1,
        talker_ns_robot2,

        # --- 重映射示例 ---
        talker_remapped,
        listener_remapped,

        # --- GroupAction 示例 ---
        robot3_group,
    ])
