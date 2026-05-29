#!/usr/bin/env python3
"""
=============================================================================
 ③ Reentrant 回调组演示 —— 同组回调可并发执行
=============================================================================

核心知识点：
  - ReentrantCallbackGroup：同一组内的回调可以并发执行
  - 必须搭配 MultiThreadedExecutor 才能真正并发
  - 用途：独立的计算任务、可以并行处理的服务调用
  - 注意：使用 Reentrant 时必须自行保证线程安全！

本示例模拟场景：
  - 使用 ReentrantCallbackGroup 让多个回调可以并发
  - 对比 MutuallyExclusiveCallbackGroup 和 ReentrantCallbackGroup 的行为差异
  - 展示线程安全问题的产生和解决方法
=============================================================================
"""

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class ReentrantDemoNode(Node):
    """演示 ReentrantCallbackGroup 的节点"""

    def __init__(self):
        super().__init__('reentrant_demo_node')

        # ============================================================
        # 1. 创建两种回调组
        # ============================================================
        # Reentrant：同组回调可并发
        self.reentrant_group = ReentrantCallbackGroup()
        # MutuallyExclusive：同组回调串行（作为对照）
        self.exclusive_group = MutuallyExclusiveCallbackGroup()

        # ============================================================
        # 2. 使用 ReentrantCallbackGroup 的定时器
        #    两个定时器回调可以同时执行
        # ============================================================

        # 定时器1 —— 在 reentrant_group 中
        self.timer1 = self.create_timer(
            1.0,
            self.timer1_callback,
            callback_group=self.reentrant_group,
        )

        # 定时器2 —— 也在 reentrant_group 中
        self.timer2 = self.create_timer(
            1.0,
            self.timer2_callback,
            callback_group=self.reentrant_group,
        )

        # 定时器3 —— 在 exclusive_group 中（对照）
        self.timer3 = self.create_timer(
            1.0,
            self.timer3_callback,
            callback_group=self.exclusive_group,
        )

        # ============================================================
        # 3. 线程安全计数器
        #    在 Reentrant 回调组中，多个回调可能同时访问共享数据
        #    必须使用锁来保护
        # ============================================================
        self.counter = 0                    # 不安全的计数器（演示竞态）
        self.safe_counter = 0               # 安全的计数器（使用锁保护）
        self.lock = threading.Lock()        # 互斥锁

        self.get_logger().info('=' * 60)
        self.get_logger().info('ReentrantCallbackGroup 演示启动')
        self.get_logger().info('  timer1 和 timer2 在 ReentrantCallbackGroup 中 → 可并发')
        self.get_logger().info('  timer3 在 MutuallyExclusiveCallbackGroup 中 → 串行')
        self.get_logger().info('  使用 MultiThreadedExecutor')
        self.get_logger().info('=' * 60)

    def timer1_callback(self):
        """定时器1回调 —— Reentrant，可与其他回调并发"""
        thread_id = threading.current_thread().name
        self.get_logger().info(
            f'[timer1] 开始, 线程: {thread_id}'
        )

        # 模拟耗时操作
        time.sleep(0.5)

        # --- 不安全的计数操作（演示竞态条件）---
        # 读取 → 修改 → 写回，不是原子操作
        temp = self.counter
        time.sleep(0.01)  # 增加竞态窗口
        temp += 1
        self.counter = temp

        # --- 安全的计数操作（使用锁保护）---
        with self.lock:
            self.safe_counter += 1

        self.get_logger().info(
            f'[timer1] 完成, 不安全计数={self.counter}, '
            f'安全计数={self.safe_counter}'
        )

    def timer2_callback(self):
        """定时器2回调 —— 与 timer1 在同一个 Reentrant 组，可并发"""
        thread_id = threading.current_thread().name
        self.get_logger().info(
            f'[timer2] 开始, 线程: {thread_id}'
        )

        time.sleep(0.3)

        # 同样的不安全操作
        temp = self.counter
        time.sleep(0.01)
        temp += 1
        self.counter = temp

        # 安全操作
        with self.lock:
            self.safe_counter += 1

        self.get_logger().info(
            f'[timer2] 完成, 不安全计数={self.counter}, '
            f'安全计数={self.safe_counter}'
        )

    def timer3_callback(self):
        """定时器3回调 —— 在 exclusive_group 中"""
        thread_id = threading.current_thread().name
        self.get_logger().info(
            f'[timer3] 开始, 线程: {thread_id}'
        )
        time.sleep(0.1)
        self.get_logger().info(f'[timer3] 完成')


def main(args=None):
    rclpy.init(args=args)

    node = ReentrantDemoNode()

    # ============================================================
    # 4. 必须使用 MultiThreadedExecutor 才能让 Reentrant 组真正并发
    #    如果用 SingleThreadedExecutor，Reentrant 也无法并发
    # ============================================================
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'\n最终统计:\n'
            f'  不安全计数 (可能有竞态): {node.counter}\n'
            f'  安全计数 (锁保护): {node.safe_counter}\n'
            f'  如果不安全计数 < 安全计数，说明发生了竞态条件'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
