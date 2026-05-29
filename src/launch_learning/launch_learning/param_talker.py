#!/usr/bin/env python3
"""
param_talker.py
带参数的发布者节点 — 用于演示 launch 中的参数传递

功能：
- 声明并使用 ROS2 参数
- 按指定频率发布消息
- 支持通过 launch 文件或命令行设置参数

参数：
- topic_name: 发布的话题名称（默认 'chatter'）
- publish_freq: 发布频率 Hz（默认 1.0）
- message_prefix: 消息前缀（默认 '[param_talker]'）

运行方式：
    ros2 run launch_learning param_talker
    ros2 run launch_learning param_talker --ros-args -p topic_name:=my_topic -p publish_freq:=5.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParamTalker(Node):
    """带参数的发布者节点"""

    def __init__(self):
        super().__init__('param_talker')

        # ============================================================
        # 1. 声明参数（declare_parameter）
        #    声明后才能通过 launch 或命令行设置
        #    第一个参数：参数名
        #    第二个参数：默认值
        # ============================================================
        self.declare_parameter('topic_name', 'chatter')
        self.declare_parameter('publish_freq', 1.0)
        self.declare_parameter('message_prefix', '[param_talker]')

        # ============================================================
        # 2. 获取参数值
        #    get_parameter() 返回 Parameter 对象，.value 获取实际值
        # ============================================================
        topic_name = self.get_parameter('topic_name').value
        publish_freq = self.get_parameter('publish_freq').value
        self.message_prefix = self.get_parameter('message_prefix').value

        # ============================================================
        # 3. 使用参数创建发布者和定时器
        # ============================================================
        self.publisher = self.create_publisher(String, topic_name, 10)
        period = 1.0 / publish_freq  # 周期 = 1 / 频率
        self.timer = self.create_timer(period, self.timer_callback)
        self.count = 0

        self.get_logger().info(
            f'参数发布者启动: topic="{topic_name}", '
            f'freq={publish_freq}Hz, prefix="{self.message_prefix}"'
        )

    def timer_callback(self):
        """定时器回调：发布消息"""
        msg = String()
        msg.data = f'{self.message_prefix} Hello #{self.count}'
        self.publisher.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ParamTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
