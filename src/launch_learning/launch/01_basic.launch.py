"""
01_basic.launch.py
ROS2 Launch 基础示例

知识点：
- LaunchDescription 的结构
- Node 动作的基本用法
- 多节点同时启动

运行方式：
    ros2 launch launch_learning 01_basic.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    generate_launch_description 是每个 launch 文件必须定义的函数。
    ros2 launch 命令会调用此函数获取 LaunchDescription 对象。

    返回值：LaunchDescription 对象，包含要执行的所有动作（Action）
    """

    # ============================================================
    # 1. LaunchDescription：launch 文件的顶层容器
    #    包含一组按顺序执行的 Action（动作）
    # ============================================================
    return LaunchDescription([

        # ============================================================
        # 2. Node：最常用的 Action，用于启动 ROS2 节点
        #    参数说明：
        #      - package: 节点所在的功能包名
        #      - executable: 可执行文件名（package.xml 中定义的）
        #      - name: 节点启动后的名称（覆盖节点代码中的名称）
        #      - output: 日志输出方式 'screen' | 'log'
        # ============================================================
        Node(
            package='launch_learning',
            executable='param_talker',
            name='my_talker',          # 自定义节点名
            output='screen',           # 日志输出到终端
        ),

        # ============================================================
        # 3. 启动第二个节点
        #    两个节点会同时启动（并行执行）
        # ============================================================
        Node(
            package='launch_learning',
            executable='ns_listener',
            name='my_listener',
            output='screen',
        ),

        # ============================================================
        # 4. 也可以启动其他包中的节点
        #    这里演示启动 ROS2 自带的 demo 节点
        # ============================================================
        # Node(
        #     package='demo_nodes_cpp',
        #     executable='talker',
        #     name='demo_talker',
        #     output='screen',
        # ),
    ])
