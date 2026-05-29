#!/usr/bin/env python3
"""
=============================================================================
 ② MutuallyExclusive 回调组演示 —— 同组回调互斥执行
=============================================================================

核心知识点：
  - MutuallyExclusiveCallbackGroup：同一组内的回调不能并发执行
  - 即使使用 MultiThreadedExecutor，同一组内仍然串行执行
  - 用途：保护共享资源（变量、硬件接口等），避免竞态条件
  - 不同组之间可以并发执行

本示例模拟场景：
  - 订阅两个话题，分别放入同一个 MutuallyExclusiveCallbackGroup
  - 使用 MultiThreadedExecutor
  - 观察同一组内的两个订阅回调互斥执行
  - 与不同组的情况做对比
=============================================================================
"""

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class MutuallyExclusiveDemoNode(Node):
    """演示 MutuallyExclusiveCallbackGroup 的节点"""

    def __init__(self):
        super().__init__('mutually_exclusive_demo_node')

        # ============================================================
        # 1. 创建一个 MutuallyExclusiveCallbackGroup
        #    同一组内的回调不能并发执行 —— 串行保护
        # ============================================================
        self.exclusive_group = MutuallyExclusiveCallbackGroup()

        # 另一个独立的回调组 —— 用于对比
        self.another_group = MutuallyExclusiveCallbackGroup()

        # ============================================================
        # 2. 将两个订阅放入同一个互斥回调组
        #    即使 MultiThreadedExecutor 有多个线程，
        #    这两个回调也不会同时执行
        # ============================================================

        # 订阅话题A —— 在 exclusive_group 中
        self.sub_a = self.create_subscription(
            String,
            '/topic_a',
            self.callback_a,
            10,
            callback_group=self.exclusive_group,  # 指定回调组
        )

        # 订阅话题B —— 也在 exclusive_group 中
        self.sub_b = self.create_subscription(
            String,
            '/topic_b',
            self.callback_b,
            10,
            callback_group=self.exclusive_group,  # 同一个组
        )

        # 订阅话题C —— 在另一个独立回调组中
        self.sub_c = self.create_subscription(
            String,
            '/topic_c',
            self.callback_c,
            10,
            callback_group=self.another_group,  # 不同的组
        )

        # ============================================================
        # 3. 创建定时器发布消息（模拟数据源）
        # ============================================================
        self.pub_a = self.create_publisher(String, '/topic_a', 10)
        self.pub_b = self.create_publisher(String, '/topic_b', 10)
        self.pub_c = self.create_publisher(String, '/topic_c', 10)

        # 发布定时器（使用独立回调组，避免阻塞）
        self.pub_timer = self.create_timer(1.0, self.publish_messages)

        # 计数器
        self.count = 0

        # 共享资源 —— 受 MutuallyExclusiveCallbackGroup 保护
        self.shared_data = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('MutuallyExclusiveCallbackGroup 演示启动')
        self.get_logger().info('  /topic_a 和 /topic_b 在同一互斥回调组')
        self.get_logger().info('  /topic_c 在另一个独立回调组')
        self.get_logger().info('  使用 MultiThreadedExecutor')
        self.get_logger().info('  预期: A和B不会并发，A/B和C可以并发')
        self.get_logger().info('=' * 60)

    def publish_messages(self):
        """定时发布消息"""
        self.count += 1
        msg_a = String()
        msg_a.data = f'msg_A_{self.count}'
        msg_b = String()
        msg_b.data = f'msg_B_{self.count}'
        msg_c = String()
        msg_c.data = f'msg_C_{self.count}'
        self.pub_a.publish(msg_a)
        self.pub_b.publish(msg_b)
        self.pub_c.publish(msg_c)

    def callback_a(self, msg: String):
        """话题A的回调 —— 在 exclusive_group 中"""
        thread_id = threading.current_thread().name
        self.get_logger().info(
            f'[callback_a] 收到: {msg.data}, 线程: {thread_id}, '
            f'共享数据: {self.shared_data}'
        )
        # 模拟耗时处理
        self.shared_data += 1
        time.sleep(0.5)
        self.get_logger().info(
            f'[callback_a] 处理完成, 共享数据更新为: {self.shared_data}'
        )

    def callback_b(self, msg: String):
        """话题B的回调 —— 也在 exclusive_group 中"""
        thread_id = threading.current_thread().name
        self.get_logger().info(
            f'[callback_b] 收到: {msg.data}, 线程: {thread_id}, '
            f'共享数据: {self.shared_data}'
        )
        # 由于和 callback_a 在同一个 MutuallyExclusiveCallbackGroup，
        # 所以 callback_a 执行期间，callback_b 必须等待
        self.shared_data += 10
        time.sleep(0.3)
        self.get_logger().info(
            f'[callback_b] 处理完成, 共享数据更新为: {self.shared_data}'
        )

    def callback_c(self, msg: String):
        """话题C的回调 —— 在 another_group 中"""
        thread_id = threading.current_thread().name
        self.get_logger().info(
            f'[callback_c] 收到: {msg.data}, 线程: {thread_id}'
        )
        # 这个回调在另一个回调组，可以和 callback_a / callback_b 并发执行
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    node = MutuallyExclusiveDemoNode()

    # ============================================================
    # 4. 使用 MultiThreadedExecutor
    #    多个线程可以同时执行不同回调组的回调
    #    但同一 MutuallyExclusiveCallbackGroup 内的回调仍然串行
    # ============================================================
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
