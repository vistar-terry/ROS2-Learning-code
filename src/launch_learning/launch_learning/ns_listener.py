#!/usr/bin/env python3
"""
ns_listener.py
带命名空间感知的订阅者节点 — 用于演示 launch 中的命名空间和重映射

功能：
- 订阅指定话题并打印消息
- 自动感知自身命名空间
- 支持通过 launch 设置话题重映射

参数：
- topic_name: 订阅的话题名称（默认 'chatter'）

运行方式：
    ros2 run launch_learning ns_listener
    ros2 run launch_learning ns_listener --ros-args -p topic_name:=my_topic
    ros2 run launch_learning ns_listener --remap chatter:=my_topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NsListener(Node):
    """带命名空间感知的订阅者节点"""

    def __init__(self):
        super().__init__('ns_listener')

        # 声明参数
        self.declare_parameter('topic_name', 'chatter')
        topic_name = self.get_parameter('topic_name').value

        # 创建订阅者
        self.subscription = self.create_subscription(
            String, topic_name, self.message_callback, 10)

        # 获取当前命名空间（launch中设置的）
        self.get_logger().info(
            f'订阅者启动: namespace="{self.get_namespace()}", '
            f'topic="{topic_name}", '
            f'节点全名="{self.get_fully_qualified_name()}"'
        )

    def message_callback(self, msg):
        """消息回调：打印收到的消息"""
        self.get_logger().info(
            f'[{self.get_namespace()}] 收到: "{msg.data}"'
        )


def main(args=None):
    rclpy.init(args=args)
    node = NsListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
