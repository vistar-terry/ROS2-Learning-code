#!/usr/bin/env python3
"""
=============================================================================
 ④ 执行器类型对比 —— SingleThreaded vs MultiThreaded vs SingleThreaded(自定义)
=============================================================================

核心知识点：
  - Executor 是 ROS2 中负责调度和执行回调的核心组件
  - SingleThreadedExecutor：单线程依次执行所有就绪回调
  - MultiThreadedExecutor：多线程并发执行不同回调组的回调
  - 执行器与回调组的配合决定了回调的执行方式

三种执行方式对比：
  ┌────────────────────┬──────────────────────┬───────────────────────┐
  │                    │ MutuallyExclusive    │ Reentrant             │
  ├────────────────────┼──────────────────────┼───────────────────────┤
  │ SingleThreaded     │ 串行（默认）         │ 串行（单线程无法并发）│
  │ MultiThreaded      │ 同组串行，不同组并发 │ 同组可并发            │
  └────────────────────┴──────────────────────┴───────────────────────┘

本示例：
  - 同一个节点配置两种回调组
  - 分别用 SingleThreadedExecutor 和 MultiThreadedExecutor 运行
  - 观察执行顺序和并发行为差异
=============================================================================
"""

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor


class ExecutorTestNode(Node):
    """用于测试不同执行器行为的节点"""

    def __init__(self):
        super().__init__('executor_test_node')

        # ============================================================
        # 1. 创建两种回调组
        # ============================================================
        self.group_a = MutuallyExclusiveCallbackGroup()
        self.group_b = ReentrantCallbackGroup()

        # ============================================================
        # 2. 创建多个定时器，分布在不同回调组
        # ============================================================

        # 定时器A1 —— 在 MutuallyExclusive 回调组 A 中
        self.timer_a1 = self.create_timer(
            1.0, self.callback_a1,
            callback_group=self.group_a,
        )

        # 定时器A2 —— 也在回调组 A 中（与 A1 互斥）
        self.timer_a2 = self.create_timer(
            1.0, self.callback_a2,
            callback_group=self.group_a,
        )

        # 定时器B1 —— 在 Reentrant 回调组 B 中
        self.timer_b1 = self.create_timer(
            1.0, self.callback_b1,
            callback_group=self.group_b,
        )

        # 定时器B2 —— 也在回调组 B 中（与 B1 可并发）
        self.timer_b2 = self.create_timer(
            1.0, self.callback_b2,
            callback_group=self.group_b,
        )

        self.count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('执行器类型对比演示启动')
        self.get_logger().info('  A1, A2 在 MutuallyExclusiveCallbackGroup → 同组串行')
        self.get_logger().info('  B1, B2 在 ReentrantCallbackGroup → 同组可并发')
        self.get_logger().info('=' * 60)

    def callback_a1(self):
        """MutuallyExclusive 组回调 A1"""
        thread_name = threading.current_thread().name
        self.count += 1
        self.get_logger().info(
            f'[A1] 开始 #{self.count}, 线程: {thread_name}'
        )
        time.sleep(0.3)
        self.get_logger().info(f'[A1] 完成 #{self.count}')

    def callback_a2(self):
        """MutuallyExclusive 组回调 A2"""
        thread_name = threading.current_thread().name
        self.count += 1
        self.get_logger().info(
            f'[A2] 开始 #{self.count}, 线程: {thread_name}'
        )
        time.sleep(0.2)
        self.get_logger().info(f'[A2] 完成 #{self.count}')

    def callback_b1(self):
        """Reentrant 组回调 B1"""
        thread_name = threading.current_thread().name
        self.count += 1
        self.get_logger().info(
            f'[B1] 开始 #{self.count}, 线程: {thread_name}'
        )
        time.sleep(0.3)
        self.get_logger().info(f'[B1] 完成 #{self.count}')

    def callback_b2(self):
        """Reentrant 组回调 B2"""
        thread_name = threading.current_thread().name
        self.count += 1
        self.get_logger().info(
            f'[B2] 开始 #{self.count}, 线程: {thread_name}'
        )
        time.sleep(0.2)
        self.get_logger().info(f'[B2] 完成 #{self.count}')


def run_with_executor(node, executor, executor_name):
    """使用指定执行器运行节点，运行5秒后停止"""
    executor.add_node(node)
    node.get_logger().info(f'\n>>> 使用 {executor_name} <<<')

    # 运行5秒
    start_time = time.time()
    while time.time() - start_time < 5.0:
        executor.spin_once(timeout_sec=0.1)

    executor.remove_node(node)


def main(args=None):
    rclpy.init(args=args)

    # ============================================================
    # 3. 对比1：SingleThreadedExecutor
    #    所有回调串行执行，无论回调组类型
    #    MutuallyExclusive 和 Reentrant 无区别（因为只有一个线程）
    # ============================================================
    node1 = ExecutorTestNode()
    single_executor = SingleThreadedExecutor()
    run_with_executor(node1, single_executor, 'SingleThreadedExecutor')
    node1.destroy_node()

    print('\n' + '=' * 60)
    print('切换到 MultiThreadedExecutor，等待3秒...')
    print('=' * 60 + '\n')
    time.sleep(3)

    # ============================================================
    # 4. 对比2：MultiThreadedExecutor
    #    不同回调组的回调可以并发
    #    同一 MutuallyExclusive 组的回调仍然串行
    #    同一 Reentrant 组的回调可以并发
    # ============================================================
    node2 = ExecutorTestNode()
    multi_executor = MultiThreadedExecutor(num_threads=4)
    run_with_executor(node2, multi_executor, 'MultiThreadedExecutor (4线程)')
    node2.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
