"""
02_parameters.launch.py
ROS2 Launch 参数传递示例

知识点：
- LaunchConfiguration：launch 参数（运行时从命令行传入）
- DeclareLaunchArgument：声明 launch 参数
- 参数默认值和描述
- 将 launch 参数传递给节点参数
- 从 YAML 文件加载参数

运行方式：
    ros2 launch launch_learning 02_parameters.launch.py
    ros2 launch launch_learning 02_parameters.launch.py topic_name:=my_topic freq:=5.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # 1. DeclareLaunchArgument：声明一个 launch 参数
    #    声明后可以通过命令行传入值
    #    参数说明：
    #      - name: 参数名（命令行传入时的键名）
    #      - default_value: 默认值（不传时使用）
    #      - description: 参数描述（帮助信息）
    # ============================================================
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='chatter',
        description='发布者使用的话题名称'
    )

    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='2.0',
        description='发布频率（Hz）'
    )

    prefix_arg = DeclareLaunchArgument(
        'message_prefix',
        default_value='[launch_param]',
        description='消息前缀字符串'
    )

    # ============================================================
    # 2. LaunchConfiguration：获取 launch 参数的值
    #    它是一个 Substitution（替换），在运行时才被解析为实际值
    #    不能在 Python 中直接当作字符串使用！
    #    必须传给支持 Substitution 的 API（如 Node 的 parameters）
    # ============================================================
    topic_name = LaunchConfiguration('topic_name')
    freq = LaunchConfiguration('freq')
    prefix = LaunchConfiguration('message_prefix')

    # ============================================================
    # 3. 将 launch 参数传递给节点的 ROS2 参数
    #    parameters 列表中的字典值支持 LaunchConfiguration
    # ============================================================
    talker_node = Node(
        package='launch_learning',
        executable='param_talker',
        name='param_talker',
        output='screen',
        # --- 方式1：逐个传递参数（使用 LaunchConfiguration） ---
        parameters=[{
            'topic_name': topic_name,        # 映射 launch 参数 → 节点参数
            'publish_freq': freq,
            'message_prefix': prefix,
        }],
    )

    # ============================================================
    # 4. 从 YAML 文件加载参数
    #    使用 FindPackageShare 定位功能包的 share 目录
    #    yaml 路径中的参数会覆盖前面的参数（后者优先）
    # ============================================================
    # 获取功能包的 share 目录路径
    pkg_share = FindPackageShare('launch_learning').find('launch_learning')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    talker_with_yaml_node = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker_yaml',
        output='screen',
        # --- 方式2：从 YAML 文件加载参数 ---
        # parameters 接受列表，可以混用字典和文件路径
        parameters=[params_file],
    )

    # 订阅者节点（话题名称需要与发布者一致）
    listener_node = Node(
        package='launch_learning',
        executable='ns_listener',
        name='ns_listener',
        output='screen',
        parameters=[{
            'topic_name': topic_name,  # 使用相同的 launch 参数
        }],
    )

    # ============================================================
    # 5. 返回 LaunchDescription
    #    DeclareLaunchArgument 必须加入 LaunchDescription 才生效
    #    Node 和 DeclareLaunchArgument 混合放入列表
    # ============================================================
    return LaunchDescription([
        # 先声明所有参数
        topic_name_arg,
        freq_arg,
        prefix_arg,
        # 再启动节点
        talker_node,
        # talker_with_yaml_node,  # 取消注释可同时启动 YAML 参数版本
        listener_node,
    ])
