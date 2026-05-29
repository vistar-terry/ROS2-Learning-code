#!/usr/bin/env python3
"""
=============================================================================
 ⑤ 多节点单执行器 —— 一个执行器管理多个节点
=============================================================================

核心知识点：
  - 一个 Executor 可以管理多个节点
  - rclpy.spin_once() vs rclpy.spin() 的区别
  - 手动执行器循环：可以插入自定义逻辑
  - 多节点不同回调组的交互

本示例模拟场景：
  - 传感器节点：发布传感器数据
  - 处理节点：订阅数据并处理
  - 两个节点在同一个 MultiThreadedExecutor 中运行
  - 展示手动执行器循环的用法
=============================================================================
"""

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class SensorNode(Node):
    """模拟传感器节点 —— 发布激光雷达数据"""

    def __init__(self):
        super().__init__('sensor_node')

        # 使用 Reentrant 回调组，允许发布和定时器并发
        self.reentrant_group = ReentrantCallbackGroup()

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.status_pub = self.create_publisher(String, '/sensor_status', 10)

        # 模拟传感器数据发布
        self.scan_timer = self.create_timer(
            0.1,  # 10Hz
            self.publish_scan,
            callback_group=self.reentrant_group,
        )

        # 状态发布
        self.status_timer = self.create_timer(
            1.0,
            self.publish_status,
            callback_group=self.reentrant_group,
        )

        self.scan_count = 0
        self.get_logger().info('[传感器节点] 启动，10Hz发布激光数据')

    def publish_scan(self):
        """发布模拟激光扫描数据"""
        self.scan_count += 1
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 0.0174533  # ~1度
        msg.range_min = 0.1
        msg.range_max = 10.0
        # 模拟360个射线
        msg.ranges = [1.0 + 0.1 * (i % 10) for i in range(360)]
        self.scan_pub.publish(msg)

    def publish_status(self):
        """发布传感器状态"""
        msg = String()
        msg.data = f'在线, 已发布 {self.scan_count} 帧扫描'
        self.status_pub.publish(msg)
        self.get_logger().info(f'[传感器] 状态: {msg.data}')


class ProcessingNode(Node):
    """数据处理节点 —— 订阅传感器数据并处理"""

    def __init__(self):
        super().__init__('processing_node')

        # ============================================================
        # 1. 回调组设计
        #    - 订阅回调使用 MutuallyExclusive：保护共享的障碍物列表
        #    - 处理定时器使用 Reentrant：可以与订阅并发
        # ============================================================
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        self.processing_group = ReentrantCallbackGroup()

        # 订阅激光数据
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.subscriber_group,
        )

        # 订阅传感器状态
        self.status_sub = self.create_subscription(
            String, '/sensor_status', self.status_callback, 10,
            callback_group=self.subscriber_group,  # 同一个互斥组
        )

        # 处理定时器
        self.process_timer = self.create_timer(
            0.5, self.process_callback,
            callback_group=self.processing_group,
        )

        # 共享数据（受 subscriber_group 的 MutuallyExclusive 保护）
        self.obstacle_count = 0
        self.sensor_online = False

        self.get_logger().info('[处理节点] 启动')

    def scan_callback(self, msg: LaserScan):
        """处理激光扫描数据 —— 在 subscriber_group 中"""
        # 统计障碍物数量（距离小于阈值的点）
        threshold = 0.5
        count = sum(1 for r in msg.ranges if r < threshold and r > msg.range_min)
        self.obstacle_count = count
        # 不打印日志，10Hz太频繁

    def status_callback(self, msg: String):
        """处理传感器状态 —— 也在 subscriber_group 中"""
        # 与 scan_callback 在同一个互斥回调组，
        # 所以不会与 scan_callback 并发执行，共享数据安全
        self.sensor_online = '在线' in msg.data
        self.get_logger().info(f'[处理] 传感器状态更新: {msg.data}')

    def process_callback(self):
        """定时处理回调 —— 在 processing_group 中"""
        if self.sensor_online:
            self.get_logger().info(
                f'[处理] 障碍物点数: {self.obstacle_count}'
            )
        else:
            self.get_logger().warn('[处理] 传感器离线，无法处理')


def main(args=None):
    rclpy.init(args=args)

    sensor_node = SensorNode()
    processing_node = ProcessingNode()

    # ============================================================
    # 2. 一个 MultiThreadedExecutor 管理两个节点
    #    两个节点的回调可以交叉并发
    #    但同一 MutuallyExclusive 回调组内的回调仍然串行
    # ============================================================
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(sensor_node)
    executor.add_node(processing_node)

    try:
        # ============================================================
        # 3. 手动执行器循环（替代 rclpy.spin）
        #    可以在循环中插入自定义逻辑
        # ============================================================
        while rclpy.ok():
            # spin_once 执行一批就绪的回调
            executor.spin_once(timeout_sec=0.1)

            # 可以在这里插入自定义逻辑
            # 例如：检查节点状态、执行非ROS任务等

    except KeyboardInterrupt:
        pass
    finally:
        sensor_node.destroy_node()
        processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
