"""
advanced/09_xml_yaml.launch.py
ROS2 Launch XML/YAML 格式与 Python 格式对比示例

知识点：
- launch 文件的三种格式：Python / XML / YAML
- XML 格式的 launch 文件写法
- YAML 格式的 launch 文件写法
- 不同格式的优缺点对比
- 格式选择建议

注意：本文件是 Python 格式的 launch 文件
       XML 和 YAML 格式的示例以注释形式给出

运行方式：
    ros2 launch launch_learning 09_xml_yaml.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    """
    ============================================================
    Launch 文件的三种格式对比
    ============================================================

    ┌──────────┬─────────────────┬──────────────────┬──────────────────┐
    │ 特性      │ Python (.py)    │ XML (.xml)       │ YAML (.yaml)     │
    ├──────────┼─────────────────┼──────────────────┼──────────────────┤
    │ 灵活性    │ ★★★★★          │ ★★★              │ ★★               │
    │ 可读性    │ ★★★             │ ★★★★             │ ★★★★★            │
    │ 条件逻辑  │ 完整Python      │ 有限             │ 有限             │
    │ 事件处理  │ 完整支持        │ 有限             │ 不支持            │
    │ 自定义Action│ 支持           │ 不支持            │ 不支持            │
    │ 学习难度  │ 较高            │ 较低             │ 最低              │
    │ 调试能力  │ 强              │ 弱               │ 弱               │
    │ 推荐场景  │ 复杂系统        │ 简单启动          │ 简单参数配置       │
    └──────────┴─────────────────┴──────────────────┴──────────────────┘

    推荐策略：
    - 简单场景：XML 或 YAML（上手快）
    - 生产系统：Python（功能完整）
    - 多机器人/复杂逻辑：Python（唯一选择）
    """

    # ============================================================
    # 以下给出等价的三种格式写法
    # 目标：启动一个 talker 节点，传入 topic_name 参数
    # ============================================================

    # --- Python 格式（本文件）---
    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='chatter',
        description='话题名称'
    )

    talker_node = Node(
        package='launch_learning',
        executable='param_talker',
        name='talker',
        output='screen',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
        }],
    )

    # ============================================================
    # --- XML 格式等价写法 ---
    # 保存为 .launch.xml 文件即可使用
    # ============================================================
    """
    <launch>
      <arg name="topic_name" default="chatter" description="话题名称"/>

      <node pkg="launch_learning" exec="param_talker" name="talker" output="screen">
        <param name="topic_name" value="$(var topic_name)"/>
      </node>
    </launch>
    """

    # ============================================================
    # --- YAML 格式等价写法 ---
    # 保存为 .launch.yaml 文件即可使用
    # ============================================================
    """
    launch:
      - arg:
          name: topic_name
          default: chatter
      - node:
          pkg: launch_learning
          exec: param_talker
          name: talker
          param:
            - name: topic_name
              value: $(var topic_name)
    """

    return LaunchDescription([
        topic_arg,
        talker_node,
    ])
