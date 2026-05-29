#!/usr/bin/env python3
"""
=============================================================================
 ① 默认行为演示 —— 单线程执行器下的回调阻塞问题
=============================================================================

核心知识点：
  - ROS2 默认使用 SingleThreadedExecutor（单线程执行器）
  - 节点的所有回调默认属于同一个 MutuallyExclusiveCallbackGroup
  - 这意味着同一时刻只能执行一个回调，其余回调必须排队等待
  - 如果某个回调执行时间较长（如耗时计算、阻塞IO），会导致其他回调被延迟

本示例模拟场景：
  - 一个定时回调每500ms执行一次，模拟耗时2秒的处理
  - 另一个定时回调每500ms执行一次，是快速处理
  - 观察在默认回调组下，长时间回调如何阻塞快速回调的执行
=============================================================================
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class DefaultBehaviorNode(Node):
    """演示默认回调组行为的节点"""

    def __init__(self):
        super().__init__('default_behavior_node')

        # ============================================================
        # 1. 查看节点默认回调组
        #    每个节点都有一个默认回调组，类型为 MutuallyExclusiveCallbackGroup
        # ============================================================
        default_group = self.default_callback_group
        self.get_logger().info(f'默认回调组类型: {type(default_group).__name__}')
        self.get_logger().info(f'是否为 MutuallyExclusiveCallbackGroup: '
                               f'{isinstance(default_group, MutuallyExclusiveCallbackGroup)}')

        # ============================================================
        # 2. 创建两个定时器 —— 都使用默认回调组（不指定 callback_group）
        #    因为默认回调组是 MutuallyExclusive 类型，
        #    所以 slow_callback 执行时，fast_callback 会被阻塞
        # ============================================================

        # 慢回调：模拟耗时处理（2秒）
        self.slow_timer = self.create_timer(
            2.0,           # 每2秒触发一次
            self.slow_callback,
            # 注意：没有指定 callback_group，默认使用节点的 default_callback_group
        )

        # 快回调：轻量处理
        self.fast_timer = self.create_timer(
            0.5,           # 每0.5秒触发一次
            self.fast_callback,
            # 同样使用默认回调组
        )

        # 计数器
        self.slow_count = 0
        self.fast_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('默认行为演示启动')
        self.get_logger().info('  慢回调: 每2秒触发，执行耗时2秒')
        self.get_logger().info('  快回调: 每0.5秒触发，执行耗时<1ms')
        self.get_logger().info('  回调组: 两个回调都在默认 MutuallyExclusiveCallbackGroup 中')
        self.get_logger().info('  预期: 慢回调执行期间，快回调被阻塞，无法按时执行')
        self.get_logger().info('=' * 60)

    def slow_callback(self):
        """慢回调 —— 模拟耗时2秒的处理"""
        self.slow_count += 1
        start_time = time.time()
        self.get_logger().warn(
            f'[慢回调 #{self.slow_count}] 开始执行 (t={start_time:.2f})'
        )

        # 模拟耗时处理（例如：图像处理、复杂计算、阻塞IO）
        time.sleep(2.0)

        elapsed = time.time() - start_time
        self.get_logger().warn(
            f'[慢回调 #{self.slow_count}] 执行完成，耗时 {elapsed:.2f}s'
        )

    def fast_callback(self):
        """快回调 —— 轻量处理"""
        self.fast_count += 1
        self.get_logger().info(
            f'[快回调 #{self.fast_count}] 执行 (t={time.time():.2f})'
        )


def main(args=None):
    rclpy.init(args=args)

    node = DefaultBehaviorNode()

    # ============================================================
    # 3. 使用默认的 SingleThreadedExecutor
    #    rclpy.spin() 内部创建 SingleThreadedExecutor
    #    单线程依次执行回调 —— 同一 MutuallyExclusiveCallbackGroup 中的回调互斥
    # ============================================================
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'\n统计: 慢回调 {node.slow_count} 次, 快回调 {node.fast_count} 次\n'
            f'注意：快回调的触发次数远少于预期（0.5s间隔），因为被慢回调阻塞了'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
